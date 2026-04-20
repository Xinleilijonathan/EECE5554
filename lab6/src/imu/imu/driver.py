#!/usr/bin/env python3
import math
import time

import serial
from transforms3d.euler import euler2quat

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
from imu_msgs.msg import Vectornav


def compute_checksum(sentence: str) -> str:
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return format(cs, "02X")


def set_imu_rate(ser: serial.Serial, hz: int = 40) -> None:
    payload = f"VNWRG,07,{hz}"
    cmd = f"${payload}*{compute_checksum(payload)}\r\n"
    ser.write(cmd.encode("utf-8"))
    time.sleep(0.1)
    ser.flushInput()


def convert_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    w, x, y_q, z = euler2quat(y, p, r, axes="rzyx")
    q = Quaternion()
    q.w, q.x, q.y, q.z = w, x, y_q, z
    return q


def parse_vnymr(line: str):
    try:
        if "*" in line:
            line = line[:line.index("*")]
        parts = line.strip().split(",")
        if parts[0] != "$VNYMR" or len(parts) < 13:
            return None
        return {
            "yaw":   float(parts[1]),
            "pitch": float(parts[2]),
            "roll":  float(parts[3]),
            "mag":   (float(parts[4]) / 10000.0,
                      float(parts[5]) / 10000.0,
                      float(parts[6]) / 10000.0),
            "accel": (float(parts[7]), float(parts[8]), float(parts[9])),
            "gyro":  (float(parts[10]), float(parts[11]), float(parts[12])),
        }
    except (ValueError, IndexError):
        return None


class VnDriverNode(Node):
    def __init__(self):
        super().__init__("vn_driver")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.pub = self.create_publisher(Vectornav, "/imu", 10)
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Opened {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Cannot open port {port}: {e}")
            raise SystemExit(1)
        set_imu_rate(self.ser, 40)
        self.timer = self.create_timer(0.005, self.read_and_publish)

    def read_and_publish(self):
        if not self.ser.in_waiting:
            return
        try:
            raw = self.ser.readline().decode("ascii", errors="replace").strip()
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return
        if not raw.startswith("$VNYMR"):
            return
        data = parse_vnymr(raw)
        if data is None:
            self.get_logger().warn(f"Bad VNYMR: {raw}")
            return
        now = self.get_clock().now().to_msg()
        hdr = Header()
        hdr.stamp = now
        hdr.frame_id = "imu1_frame"

        imu_msg = Imu()
        imu_msg.header = hdr
        imu_msg.orientation = convert_to_quaternion(data["roll"], data["pitch"], data["yaw"])
        imu_msg.angular_velocity.x    = data["gyro"][0]
        imu_msg.angular_velocity.y    = data["gyro"][1]
        imu_msg.angular_velocity.z    = data["gyro"][2]
        imu_msg.linear_acceleration.x = data["accel"][0]
        imu_msg.linear_acceleration.y = data["accel"][1]
        imu_msg.linear_acceleration.z = data["accel"][2]
        imu_msg.orientation_covariance[0]         = -1.0
        imu_msg.angular_velocity_covariance[0]    = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        mag_msg = MagneticField()
        mag_msg.header = hdr
        mag_msg.magnetic_field.x = data["mag"][0]
        mag_msg.magnetic_field.y = data["mag"][1]
        mag_msg.magnetic_field.z = data["mag"][2]

        vn_msg = Vectornav()
        vn_msg.header    = hdr
        vn_msg.imu       = imu_msg
        vn_msg.mag_field = mag_msg
        vn_msg.raw_str   = raw
        self.pub.publish(vn_msg)

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VnDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
