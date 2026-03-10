# GNSS RTK Driver Package

This package implements a ROS2 driver for parsing GNSS RTK data from an NMEA stream.  
It is developed as part of **EECE5554 Lab 3** to demonstrate how RTK GNSS data can be parsed, processed, and published in ROS2.

The driver reads NMEA messages from a serial port, extracts positioning information from **GNGGA sentences**, and publishes the data as ROS messages.

---

## Package Structure

```
gnss_rtk
│   ├── gnss_rtk
│   │   ├── __init__.py
│   │   └── rtk_driver.py
│   ├── launch
│   │   └── rtk_driver_launch.py
│   ├── package.xml
│   ├── resource
│   │   └── gnss_rtk
│   ├── setup.cfg
│   └── setup.py
```

---

## Features

The driver performs the following tasks:

- Reads NMEA data from a serial port
- Parses **GNGGA messages**
- Extracts the following fields:
  - UTC time
  - Latitude
  - Longitude
  - Altitude
  - Fix quality
  - Number of satellites
  - HDOP
- Converts latitude/longitude to **UTM coordinates**
- Publishes GNSS data as ROS messages

---

## ROS Topics

The driver publishes GNSS data to a ROS topic containing:

- UTM Easting
- UTM Northing
- Altitude
- Fix quality
- Number of satellites
- HDOP
- Timestamp

---

## Running the Driver

To run the driver in ROS2:

```bash
ros2 run gnss_rtk rtk_driver
