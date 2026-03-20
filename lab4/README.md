# Lab 4 – IMU Data Analysis and Motion Characterization

## Overview
In this lab, we worked with an Inertial Measurement Unit (IMU) to analyze both stationary and dynamic motion data. The goal was to understand sensor behavior, noise characteristics, and how IMU data can be used for orientation estimation.

---

## Objectives
- Record IMU data using ROS2 and rosbag
- Analyze gyroscope and accelerometer measurements
- Compute Allan deviation to characterize sensor noise
- Estimate IMU noise parameters (ARW, bias instability, etc.)
- Visualize orientation (roll, pitch, yaw) over time
- Observe IMU behavior during free-form motion

---

## Folder Structure

```
lab4
├── analysis/              # Python scripts for data analysis and plotting
├── data/                  # Recorded rosbag files
├── src/imu/               # ROS2 package for IMU node
├── README.md              # This file
```

---

## What I Did

### 1. Data Collection
- Used ROS2 to record IMU data into rosbag files
- Collected:
  - **Stationary data** (for noise analysis)
  - **Free-form motion data** (for dynamic behavior analysis)

---

### 2. Sensor Data Analysis
- Plotted:
  - Gyroscope data (ωx, ωy, ωz)
  - Accelerometer data (ax, ay, az)
- Observed:
  - Noise characteristics during stationary periods
  - Spikes corresponding to motion events

---

### 3. Orientation Estimation
- Computed **roll, pitch, and yaw** from IMU data
- Observed:
  - Smooth changes during motion
  - Drift in yaw due to lack of external correction

---

### 4. Allan Deviation Analysis
- Computed Allan deviation from stationary gyro data
- Identified:
  - **Angle Random Walk (ARW)** from slope −1/2
  - Attempted to observe bias instability and rate random walk
- Found that:
  - ARW is dominant in the dataset
  - Bias instability and rate random walk are not clearly visible

---

### 5. Motion Analysis
- Extracted short motion clips from rosbag
- Compared:
  - Gyroscope spikes → rotational motion
  - Accelerometer changes → direction + gravity effects

---

## Key Observations

- Gyroscope captures **angular velocity** and shows sharp spikes during motion  
- Accelerometer captures **linear acceleration + gravity**  
- Orientation estimation:
  - Stable in short term  
  - Drift accumulates over time (especially yaw)  
- Allan deviation shows dominant **white noise (ARW)** behavior  

---

## How to Run

### 1. Build ROS2 package
```bash
colcon build
source install/setup.bash
```

### 2. Play rosbag
```bash
ros2 bag play <bag_file>
```

### 3. Run analysis scripts
```bash
python analysis/<your_script>.py
```

---

## Dependencies
- ROS2 (Jazzy or compatible)
- Python 3
- numpy
- matplotlib

---

## Conclusion
This lab provided hands-on experience with IMU data processing, noise modeling, and motion analysis. It demonstrated both the strengths and limitations of IMUs, particularly the presence of noise and drift in orientation estimation.