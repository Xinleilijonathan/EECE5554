#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import math
from transforms3d.euler import euler2quat  # pip install transforms3d

from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
from imu_msgs.msg import Vectornav           # replace with your actual package name


# ── VectorNav register write to set output rate to 40 Hz ──────────────────────
# Command: $VNWRG,07,40*XX  (Register 7 = Async Data Output Frequency)
RATE_CMD = b'$VNWRG,07,40*XX\r\n'           # checksum not required for this register


def compute_checksum(sentence: str) -> str:
    """XOR checksum of all characters between $ and * (exclusive)."""
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return format(cs, '02X')


def set_imu_rate(ser: serial.Serial, hz: int = 40) -> None:
    """Write async output frequency register (reg 7) to hz."""
    payload = f'VNWRG,07,{hz}'
    checksum = compute_checksum(payload)
    cmd = f'${payload}*{checksum}\r\n'
    ser.write(cmd.encode('utf-8'))
    time.sleep(0.1)
    # flush any response
    ser.flushInput()


def convert_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """
    Convert Euler angles (degrees) to a quaternion.
    VectorNav convention: Yaw (Z), Pitch (Y), Roll (X)  — i.e. ZYX / aerospace.
    transforms3d.euler.euler2quat uses (ai, aj, ak) with axes string.
    Returns geometry_msgs/Quaternion.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    # euler2quat returns (w, x, y, z) for axes='rzyx' (ZYX intrinsic = XYZ extrinsic)
    w, x, y_q, z = euler2quat(y, p, r, axes='rzyx')

    q = Quaternion()
    q.w = w
    q.x = x
    q.y = y_q
    q.z = z
    return q


def parse_vnymr(line: str):
    """
    Parse a $VNYMR sentence.
    Format:
      $VNYMR,<Yaw>,<Pitch>,<Roll>,<MagX>,<MagY>,<MagZ>,
             <AccX>,<AccY>,<AccZ>,<GyroX>,<GyroY>,<GyroZ>*<Checksum>
    Units from VectorNav:
      Orientation : degrees
      Mag         : Gauss  → convert to Tesla (/10000)
      Accel       : m/s²   (already SI)
      Gyro        : rad/s  (already SI)
    """
    try:
        # strip checksum
        if '*' in line:
            line = line[:line.index('*')]
        parts = line.strip().split(',')
        if parts[0] != '$VNYMR' or len(parts) < 13:
            return None

        yaw   = float(parts[1])
        pitch = float(parts[2])
        roll  = float(parts[3])
        mag_x = float(parts[4])
        mag_y = float(parts[5])
        mag_z = float(parts[6])
        acc_x = float(parts[7])
        acc_y = float(parts[8])
        acc_z = float(parts[9])
        gyr_x = float(parts[10])
        gyr_y = float(parts[11])
        gyr_z = float(parts[12])

        return {
            'yaw': yaw, 'pitch': pitch, 'roll': roll,
            'mag': (mag_x / 10000.0, mag_y / 10000.0, mag_z / 10000.0),   # Gauss → Tesla
            'accel': (acc_x, acc_y, acc_z),                                # m/s²
            'gyro':  (gyr_x, gyr_y, gyr_z),                               # rad/s
        }
    except (ValueError, IndexError):
        return None


class VnDriverNode(Node):

    def __init__(self):
        super().__init__('vn_driver')

        # ── parameters (set via launch file or CLI) ────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # ── publisher ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(Vectornav, '/imu', 10)

        # ── open serial ───────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open port {port}: {e}')
            raise SystemExit(1)

        # ── configure 40 Hz output ─────────────────────────────────────────
        set_imu_rate(self.ser, 40)
        self.get_logger().info('VectorNav configured for 40 Hz output')

        # ── read loop timer (poll faster than 40 Hz so we don't miss msgs) ─
        self.timer = self.create_timer(0.005, self.read_and_publish)

    def read_and_publish(self):
        if not self.ser.in_waiting:
            return

        try:
            raw = self.ser.readline().decode('ascii', errors='replace').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if not raw.startswith('$VNYMR'):
            return

        data = parse_vnymr(raw)
        if data is None:
            self.get_logger().warn(f'Bad VNYMR string: {raw}')
            return

        now = self.get_clock().now().to_msg()

        # ── Header ────────────────────────────────────────────────────────
        hdr = Header()
        hdr.stamp    = now
        hdr.frame_id = 'imu1_frame'

        # ── IMU message ───────────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header = hdr
        imu_msg.orientation = convert_to_quaternion(
            data['roll'], data['pitch'], data['yaw']
        )
        imu_msg.angular_velocity.x    = data['gyro'][0]
        imu_msg.angular_velocity.y    = data['gyro'][1]
        imu_msg.angular_velocity.z    = data['gyro'][2]
        imu_msg.linear_acceleration.x = data['accel'][0]
        imu_msg.linear_acceleration.y = data['accel'][1]
        imu_msg.linear_acceleration.z = data['accel'][2]
        # covariance unknown → set first element to -1 per REP-145
        imu_msg.orientation_covariance[0]         = -1.0
        imu_msg.angular_velocity_covariance[0]    = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        # ── MagneticField message ─────────────────────────────────────────
        mag_msg = MagneticField()
        mag_msg.header = hdr
        mag_msg.magnetic_field.x = data['mag'][0]
        mag_msg.magnetic_field.y = data['mag'][1]
        mag_msg.magnetic_field.z = data['mag'][2]

        # ── Compose and publish ───────────────────────────────────────────
        vn_msg = Vectornav()
        vn_msg.header    = hdr
        vn_msg.imu       = imu_msg
        vn_msg.mag_field = mag_msg
        vn_msg.raw_str   = raw

        self.pub.publish(vn_msg)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
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


if __name__ == '__main__':
    main()