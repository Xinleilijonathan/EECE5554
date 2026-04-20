# Lab 6 — NUance Multi-Sensor Navigation

EECE 5554 — Robotics Sensing and Navigation

## Overview

This lab implements multi-sensor navigation to determine the path of a moving car using the NUance autonomous vehicle platform (Ford Mustang Mach-E). Sensor modalities include GPS, IMU (VectorNav VN-100), Ouster LiDAR, and Lucid RGB camera. A Continental ARS548 radar was **not physically installed** on the vehicle during data collection.

## Repository Structure

```
lab6/
├── analysis/
│   ├── figures/                      # Generated plots (Figs 0–10, Q5 bonus)
│   │   ├── fig0_mag_scatter.png      # Magnetometer calibration before/after
│   │   ├── fig1_mag_yaw.png          # Magnetometer yaw before/after calibration
│   │   ├── fig2_gyro_yaw.png         # Gyro-integrated yaw estimate
│   │   ├── fig3_complementary_filter.png  # 4-subplot complementary filter
│   │   ├── fig4_velocity_accel.png   # Forward velocity from accelerometer
│   │   ├── fig5_velocity_gps.png     # Forward velocity from GPS
│   │   ├── fig6_trajectory.png       # GPS vs. IMU dead-reckoning trajectory
│   │   ├── fig7_lidar_snapshot_1.png # LiDAR point cloud frame 1
│   │   ├── fig7_lidar_snapshot_2.png # LiDAR point cloud frame 2
│   │   ├── fig8_compare_all.png      # LiDAR odometry + GPS + IMU overlay
│   │   ├── fig9_camera_snapshots.png # 4 representative camera frames
│   │   ├── fig10_radar_na.png        # Radar N/A placeholder
│   │   └── fig_q5_centripetal.png    # Q5 centripetal acceleration comparison
│   ├── report/
│   │   ├── report.tex                # LaTeX report source
│   │   └── report.pdf                # Compiled PDF report
│   └── scripts/
│       ├── lab6_analysis.py          # Main analysis script (all figures + stats)
│       └── inspect_bag.py            # Rosbag topic inspection utility
├── data/
│   ├── rosbags/                      # Place rosbag recordings here
│   └── snapshots/                    # Place video snapshots here
├── imu_and_gps_in_a_circle/          # Calibration dataset (circles — IMU + GPS)
│   ├── data_parking_lot_new_0.mcap
│   └── metadata.yaml
├── Around_the_city_imu_and_gps/      # City driving dataset (IMU + GPS)
│   ├── oon_the_way_0.mcap
│   └── metadata.yaml
├── src/
│   ├── gnss/                         # ROS 2 GPS puck driver package
│   ├── imu/                          # ROS 2 VectorNav VN-100 driver package
│   ├── imu_msgs/                     # Custom Vectornav message definition
│   └── launch/
│       └── lab6_launch.py            # Unified IMU + GPS launch file
├── Lab6-Nuance.ipynb                 # Lab specification and rubric
├── Lab6_complementary_filter.ipynb   # Reference: Butterworth filter examples
└── README.md
```

## Data Collection

Two rosbags were recorded using the NUance vehicle:

| Dataset | File | Contents |
|---|---|---|
| Calibration (circles) | `imu_and_gps_in_a_circle/data_parking_lot_new_0.mcap` | IMU + GPS, ~4–5 circles at Ruggles |
| City drive | `Around_the_city_imu_and_gps/oon_the_way_0.mcap` | IMU + GPS, ~2–3 km route with 10+ turns |

Topics recorded: `/imu`, `/gps`

The Ouster LiDAR (`/ouster/points`) and Lucid camera (`/lucid/image_raw`) were driven by the NUance on-board Nuvo PC. Camera snapshots from the drive are shown in Fig. 9 of the report.

**Radar:** The Continental ARS548 was not physically installed on the vehicle at the time of data collection. Fig. 10 is a placeholder noting this.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select imu_msgs gnss imu
source install/setup.bash
```

Build order matters: `imu_msgs` must be built before `imu` and `gnss` (custom message dependency).

## Launching Sensors

### On your laptop (IMU + GPS)

```bash
# Default ports
ros2 launch lab6_launch.py

# Custom ports
ros2 launch lab6_launch.py imu_port:=/dev/ttyUSB0 gps_port:=/dev/ttyUSB1
```

### On the NUance Nuvo PC (LiDAR + camera + radar)

```bash
ssh username@192.168.0.10
ros2 launch username main.launch.xml
```

### Recording a rosbag

```bash
# Calibration circles
ros2 bag record -o data_going_in_circles /gps /imu /ouster/points /lucid/image_raw

# City drive
ros2 bag record -o data_driving /gps /imu /ouster/points /lucid/image_raw
```

## Running Analysis

```bash
cd lab6/analysis/scripts
python3 lab6_analysis.py
```

All figures are saved to `analysis/figures/`. Key statistics (Q5 centripetal RMS, Q7 GPS–IMU agreement duration) are printed to stdout.

**Dependencies:** `numpy`, `scipy`, `matplotlib`, `open3d`, `cv2`, `rosbags`

Install in the provided virtual environment:

```bash
source lab6/.venv/bin/activate
```

## Analysis Summary

| Figure | Description | Status |
|---|---|---|
| Fig. 0 | Magnetometer X-Y scatter before/after hard+soft iron calibration | ✅ |
| Fig. 1 | Magnetometer yaw raw vs. calibrated | ✅ |
| Fig. 2 | Gyro-integrated yaw | ✅ |
| Fig. 3 | Complementary filter: LPF(mag), HPF(gyro), fused, IMU heading | ✅ |
| Fig. 4 | Forward velocity from accelerometer (before/after bias removal) | ✅ |
| Fig. 5 | Forward velocity from GPS | ✅ |
| Fig. 6 | Trajectory: GPS vs. IMU dead-reckoning (aligned to same start) | ✅ |
| Fig. 7 | LiDAR point cloud — 2 representative frames annotated | ✅ |
| Fig. 8 | LiDAR odometry + GPS + IMU dead-reckoning on one plot | ✅ |
| Fig. 9 | Camera snapshots — 4 frames with trajectory annotations | ✅ |
| Fig. 10 | Radar detections — N/A (sensor not installed) | ➖ |
