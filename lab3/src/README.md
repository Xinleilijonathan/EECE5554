# Lab 3 - RTK GNSS Driver

This lab focuses on implementing a **ROS2 driver for RTK GNSS data**.

The driver reads GNSS NMEA messages, extracts relevant fields, converts coordinates to UTM format, and publishes the data as a ROS2 message.

## Objectives

The RTK GNSS driver must:

- Read `GNGGA` messages from a GNSS device or emulator
- Parse the following fields:
  - UTC time
  - Latitude
  - Longitude
  - Altitude
  - Fix quality
  - HDOP
- Convert latitude and longitude to **UTM coordinates**
- Publish the parsed information using a custom ROS2 message

## Repository Layout


lab3
│
├── analysis
│ └── scripts for analyzing recorded GNSS data
│
├── data
│ └── rosbag datasets used during testing
│
└── src
└── gnss_rtk
└── ROS2 package implementing the RTK GNSS driver


## Data Source

RTK data is processed using the **sensor emulator and provided dataset files** from the course staff.

The emulator streams NMEA messages to simulate GNSS hardware.

## Build Instructions

Navigate to the lab3 workspace:

```bash
cd lab3
colcon build
source install/setup.bash

Run the Driver
ros2 launch gnss_rtk rtk_driver_launch.py
Expected Output

The node publishes RTK GNSS information including:

Latitude

Longitude

Altitude

UTM Easting

UTM Northing

Fix quality

HDOP
