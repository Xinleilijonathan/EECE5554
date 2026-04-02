# Lab 5 Source Code (ROS2 Packages)

## Overview

This directory contains ROS2 packages for handling GNSS and IMU data. Each package follows the standard ROS2 Python package structure and includes drivers, example nodes, and launch files.

---

## Packages

### 1. GNSS Package

**Path:** `src/gnss/`

**Purpose:**
Handles GNSS (GPS) data acquisition and processing.

#### Key Files

* `gnss/gnss_node.py`
  ROS2 node for reading and publishing GNSS data

* `launch/standalone_driver.launch.py`
  Launch file to start the GNSS driver/node

* `package.xml`, `setup.py`, `setup.cfg`
  ROS2 package configuration

* `resource/gnss`
  Package resource marker

* `test/`
  Contains ROS2 package test scripts

---

### 2. IMU Package

**Path:** `src/imu/`

**Purpose:**
Handles IMU sensor data (accelerometer, gyroscope, magnetometer).

#### Key Files

* `imu/driver.py`
  Core IMU driver for reading sensor data

* `imu/example_node.py`
  Example ROS2 node for publishing IMU data

* `launch/driver.launch.py`
  Launch file to run the IMU driver

* `package.xml`, `setup.py`, `setup.cfg`
  ROS2 package configuration

* `resource/imu`
  Package resource marker

* `test/`
  Contains ROS2 package test scripts

---

## Directory Structure

```bash
src/
├── gnss/
│   ├── gnss/
│   │   ├── __init__.py
│   │   └── gnss_node.py
│   ├── launch/
│   │   └── standalone_driver.launch.py
│   ├── resource/
│   │   └── gnss
│   ├── test/
│   │   ├── test_copyright.py
│   │   ├── test_flake8.py
│   │   └── test_pep257.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── imu/
│   ├── imu/
│   │   ├── __init__.py
│   │   ├── driver.py
│   │   └── example_node.py
│   ├── launch/
│   │   └── driver.launch.py
│   ├── resource/
│   │   └── imu
│   ├── test/
│   │   ├── test_copyright.py
│   │   ├── test_flake8.py
│   │   └── test_pep257.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── README.md
```

---

## Notes

* Both packages follow standard ROS2 Python package conventions
* Launch files allow easy execution of sensor drivers
* Test folders are auto-generated ROS2 package test utilities
* These packages can be extended for localization and navigation tasks
