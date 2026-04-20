#!/usr/bin/env python3
import time
import calendar
from datetime import datetime, timezone

import serial
import utm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from custom_msgs.msg import Customgps


def nmea_degmin_to_decimal(deg_min: str, direction: str) -> float:
    if not deg_min or not direction:
        raise ValueError("missing lat/lon field")
    v = float(deg_min)
    deg = int(v // 100)
    minutes = v - deg * 100.0
    dec = deg + minutes / 60.0
    if direction in ("S", "W"):
        dec *= -1.0
    return dec


def gps_utc_hhmmss_to_epoch_today(utc_hhmmss: str) -> int:
    if not utc_hhmmss or len(utc_hhmmss) < 6:
        raise ValueError("bad utc field")
    hh = int(utc_hhmmss[0:2])
    mm = int(utc_hhmmss[2:4])
    ss = int(float(utc_hhmmss[4:]))
    secs_since_midnight = hh * 3600 + mm * 60 + ss
    now_utc = datetime.now(timezone.utc)
    midnight_utc = datetime(now_utc.year, now_utc.month, now_utc.day, 0, 0, 0,
                            tzinfo=timezone.utc)
    return calendar.timegm(midnight_utc.timetuple()) + secs_since_midnight


class GNSSDriver(Node):
    def __init__(self):
        super().__init__("gnss_driver")
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 4800)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.get_logger().info(f"Opening serial port: {port} @ {baud}")
        self.ser = serial.Serial(port, baudrate=int(baud), timeout=0.1)
        self.pub = self.create_publisher(Customgps, "/gps", 10)
        self.timer = self.create_timer(0.2, self.tick)
        self.last_warn_time = 0.0

    def tick(self):
        try:
            if self.ser.in_waiting == 0:
                return
        except Exception:
            pass
        raw = self.ser.readline().decode("ascii", errors="replace").strip()
        if not raw:
            return
        if not (raw.startswith("$GPGGA") or raw.startswith("$GNGGA")):
            return
        parts = raw.split(",")
        if len(parts) < 10:
            self._throttled_warn(f"Short GGA line: {raw}")
            return
        try:
            utc = parts[1]
            lat_raw, lat_dir = parts[2], parts[3]
            lon_raw, lon_dir = parts[4], parts[5]
            fix_quality = parts[6]
            hdop_str = parts[8]
            alt_str = parts[9]
            if fix_quality == "0" or fix_quality == "":
                self._throttled_warn("No GPS fix yet (fix_quality=0)")
                return
            if not utc or not lat_raw or not lat_dir or not lon_raw or not lon_dir:
                raise ValueError("missing required GGA fields")
            lat = nmea_degmin_to_decimal(lat_raw, lat_dir)
            lon = nmea_degmin_to_decimal(lon_raw, lon_dir)
            hdop = float(hdop_str) if hdop_str else 0.0
            altitude = float(alt_str) if alt_str else 0.0
            easting, northing, zone, letter = utm.from_latlon(lat, lon)
            epoch = gps_utc_hhmmss_to_epoch_today(utc)
            msg = Customgps()
            msg.header = Header()
            msg.header.frame_id = "GPS1_Frame"
            msg.header.stamp.sec = int(epoch)
            msg.header.stamp.nanosec = 0
            msg.latitude = float(lat)
            msg.longitude = float(lon)
            msg.altitude = float(altitude)
            msg.utm_easting = float(easting)
            msg.utm_northing = float(northing)
            msg.zone = int(zone)
            msg.letter = str(letter)
            msg.hdop = float(hdop)
            msg.gpgga_read = raw
            self.pub.publish(msg)
        except Exception as e:
            self._throttled_warn(f"Parse/publish failed: {e} | RAW={raw}")

    def _throttled_warn(self, text: str, period_s: float = 1.0):
        now = time.time()
        if now - self.last_warn_time >= period_s:
            self.get_logger().warn(text)
            self.last_warn_time = now


def main():
    rclpy.init()
    node = GNSSDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
