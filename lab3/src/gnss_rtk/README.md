---

# 3 lab3/src/gnss_rtk/README.md
（ROS2 package说明）

```markdown
# gnss_rtk

ROS2 Python package implementing an **RTK GNSS driver**.

The driver reads NMEA GNGGA messages from a GNSS device (or emulator), extracts navigation information, converts geographic coordinates to UTM format, and publishes the results as a ROS2 message.

## Package Structure


gnss_rtk
│
├── gnss_rtk
│ ├── init.py
│ └── rtk_driver.py
│
├── launch
│ └── rtk_driver_launch.py
│
├── msg
│ └── Customrtk.msg
│
├── resource
│ └── gnss_rtk
│
├── package.xml
├── setup.py
└── setup.cfg


## Node

### rtk_driver

The node performs the following tasks:

1. Reads serial data from a GNSS device or emulator
2. Parses `GNGGA` messages
3. Extracts navigation fields
4. Converts latitude/longitude to UTM coordinates
5. Publishes the data as a ROS2 message

## Message Definition

Custom message: **Customrtk.msg**

Fields include:

- latitude
- longitude
- altitude
- UTM easting
- UTM northing
- zone
- zone letter
- UTC time
- raw GNGGA string
- fix quality
- HDOP

## Running the Driver

Launch the node:

```bash
ros2 launch gnss_rtk rtk_driver_launch.py
