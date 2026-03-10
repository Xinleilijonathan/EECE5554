#!/usr/bin/env python3

import math
import serial
import utm

import rclpy
from rclpy.node import Node

from gnss_rtk.msg import Customrtk


def convert_to_decimal_degrees(raw_value: str, direction: str, is_longitude: bool) -> float:
    if not raw_value:
        return 0.0

    if is_longitude:
        degrees = int(raw_value[:3])
        minutes = float(raw_value[3:])
    else:
        degrees = int(raw_value[:2])
        minutes = float(raw_value[2:])

    decimal = degrees + minutes / 60.0

    if direction in ['S', 'W']:
        decimal = -decimal

    return decimal


class RTKDriver(Node):
    def __init__(self):
        super().__init__('rtk_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 4800)
        self.declare_parameter('topic_name', '/rtk_gnss')

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        topic_name = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(Customrtk, topic_name, 10)

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Opened serial port: {port} @ {baudrate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        self.timer = self.create_timer(0.05, self.read_gnss_data)

    def read_gnss_data(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace').strip()
        except Exception as e:
            self.get_logger().warn(f'Error reading serial data: {e}')
            return

        if not line:
            return

        if not line.startswith('$GNGGA'):
            return

        fields = line.split(',')

        if len(fields) < 10:
            self.get_logger().warn(f'Incomplete GNGGA sentence: {line}')
            return

        try:
            utc = fields[1]
            lat_raw = fields[2]
            lat_dir = fields[3]
            lon_raw = fields[4]
            lon_dir = fields[5]
            fix_quality = int(fields[6]) if fields[6] else 0
            hdop = float(fields[8]) if fields[8] else 0.0
            altitude = float(fields[9]) if fields[9] else 0.0

            latitude = convert_to_decimal_degrees(lat_raw, lat_dir, is_longitude=False)
            longitude = convert_to_decimal_degrees(lon_raw, lon_dir, is_longitude=True)

            utm_easting, utm_northing, zone, letter = utm.from_latlon(latitude, longitude)

            msg = Customrtk()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'

            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude
            msg.utm_easting = float(utm_easting)
            msg.utm_northing = float(utm_northing)
            msg.zone = int(zone)
            msg.letter = str(letter)
            msg.utc = utc
            msg.gngga_read = line
            msg.fix_quality = fix_quality
            msg.hdop = hdop

            self.publisher_.publish(msg)

            self.get_logger().info(
                f'Published RTK: lat={latitude:.6f}, lon={longitude:.6f}, '
                f'alt={altitude:.2f}, fix={fix_quality}, hdop={hdop:.2f}'
            )

        except Exception as e:
            self.get_logger().warn(f'Failed to parse GNGGA line: {line} | error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RTKDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
