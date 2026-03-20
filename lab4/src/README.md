# IMU ROS2 Package

## Overview
This package implements a simple ROS2-based IMU node that publishes inertial measurement data, including angular velocity and linear acceleration. It serves as the data source for recording, analysis, and visualization in Lab 4.

---

## Purpose
The purpose of this package is to:
- Interface with IMU data (real or simulated)
- Publish IMU measurements to ROS2 topics
- Enable rosbag recording for later analysis
- Provide a modular structure for extending IMU-related functionality

---

## Package Structure

```
imu
├── imu/
│   ├── __init__.py
│   └── example_node.py       # Main IMU publisher node
├── launch/
│   └── example_launch.py     # Launch file to start the node
├── resource/
│   └── imu
├── package.xml              # Package metadata and dependencies
├── setup.cfg
├── setup.py                 # Python package setup
└── README.md                # This file
```

---

## Node Description

### `example_node.py`
This node:
- Initializes a ROS2 node
- Publishes IMU data (angular velocity and linear acceleration)
- Uses standard ROS2 message types (e.g., `sensor_msgs/msg/Imu`)
- Runs at a fixed frequency

The published data can be:
- Recorded using `ros2 bag`
- Visualized using tools like RViz2
- Used for further analysis (e.g., Allan deviation, orientation estimation)

---

## Launch File

### `example_launch.py`
This launch file:
- Starts the IMU node
- Simplifies execution without manually running the node script

Run using:
```bash
ros2 launch imu example_launch.py
```

---

## How to Run

### 1. Build the workspace
```bash
colcon build
source install/setup.bash
```

### 2. Run the node directly
```bash
ros2 run imu example_node
```

### 3. Or use the launch file
```bash
ros2 launch imu example_launch.py
```

---

## Topics

Typical published topic:
- `/imu` → IMU data (angular velocity + linear acceleration)

---

## Dependencies
- rclpy
- sensor_msgs
- ROS2 (Jazzy or compatible)

---

## Role in Lab 4
This package was used to generate and publish IMU data, which was then recorded into rosbag files. The recorded data served as the input for all subsequent analysis, including plotting sensor signals, computing Allan deviation, and estimating orientation.

---

## Notes
- The current implementation is a basic example node
- It can be extended to support real hardware drivers or more advanced filtering (e.g., sensor fusion)

---