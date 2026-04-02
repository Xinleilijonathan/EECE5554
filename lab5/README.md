# EECE 5554 - Lab 5: Inertial Odometry

## Overview

This lab investigates inertial odometry using IMU data collected from walking experiments. Two datasets were recorded: circular motion and square motion. The objective is to analyze IMU signals and understand how sensor noise and integration affect motion estimation.

---

## What was done

### Data Collection

* Collected IMU data for:

  * Circular walking trajectory
  * Square walking trajectory

### Data Processing & Analysis

* Magnetometer calibration (hard-iron offset correction)
* Gyroscope analysis:

  * Angular velocity
  * Integrated rotation
* Accelerometer analysis:

  * Acceleration
  * Velocity (single integration)
  * Displacement (double integration for additional analysis)
* Heading estimation using:

  * Magnetometer
  * Integrated gyroscope
* Trajectory reconstruction (North-East frame)

---

## Results

* Reconstructed trajectories approximately match expected shapes (circle and square)
* Significant drift observed due to:

  * Sensor noise
  * Integration error accumulation
* Demonstrates limitations of pure inertial navigation

---

## Repository Structure

```bash
lab5/
├── analysis/
│   ├── imu_analysis.py
|   └── gps_analysis.py
│   ├── report/
│   │   ├── lab5_report.tex
│   │   └── lab5_report.pdf
├── data/
│   ├── circle_walk_snapshot.png
|   └── square_walk_snapshot.png
│   └── circle/
|   └── square/
├── src/
│   ├── gnss/
│   ├── imu/
│   └── README.md
├── README.md
```

---

## Notes

* Rosbag files are not included due to size constraints
* Generated plots are used in the report but not tracked in the repository
* Focus is on analysis scripts and report
