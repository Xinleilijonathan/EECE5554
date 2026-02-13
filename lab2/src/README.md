# EECE5554 — Lab 2: GNSS Driver and Data Analysis

## Overview

This repository contains the implementation and analysis for **Lab 2**, focusing on GNSS data collection, ROS2 integration, and statistical evaluation of positioning accuracy under different environmental conditions.

The project includes:

* A ROS2 GNSS driver package
* Custom message definitions
* Recorded rosbag datasets
* Python-based analysis scripts
* Generated plots and error metrics

The goal of this lab is to evaluate GPS performance in **open**, **occluded**, and **straight-line walking** environments and quantify noise, drift, and positioning error.

---

## ROS2 GNSS Driver

The `gnss` package interfaces with the GNSS receiver and publishes GPS data to ROS2 topics.

### Key Features

* Parses GNSS data from the hardware receiver
* Publishes standardized ROS2 messages
* Provides UTM coordinates for metric-based analysis
* Designed for compatibility with ROS2 Jazzy

---

## Data Collection

Three datasets were recorded using ROS2 rosbag:

| Dataset       | Environment             | Purpose                    |
| ------------- | ----------------------- | -------------------------- |
| Open Area     | Minimal obstruction     | Baseline GPS accuracy      |
| Occluded Area | Trees / buildings       | Evaluate multipath effects |
| Straight Line | Controlled walking path | Measure noise and drift    |

---

## Analysis Pipeline

The GNSS datasets were processed using a custom Python script:

```
analyze_gnss_bags.py
```

### Processing Steps

1. Deserialize ROS2 bag messages
2. Export structured CSV files
3. Convert positions into UTM coordinates
4. Generate plots for spatial and temporal analysis
5. Compute positioning error metrics

---

## Generated Plots

The following figures were produced automatically:

### Stationary Analysis

* Northing vs Easting scatterplots (open vs occluded)
* Altitude vs time
* Error histograms

### Walking Analysis

* Straight-line trajectory
* Cross-track error
* Altitude vs time

### Quality Metrics

* HDOP vs time
* Latitude and longitude stability

---

## Noise and Drift Estimation

A PCA-based best-fit line was applied to the straight-line dataset.

### Cross-track error:

```
e = dx * (y - cy) - dy * (x - cx)
```

Where `(cx, cy)` is the centroid and `(dx, dy)` is the unit direction vector.

### Metrics computed:

* Noise → standard deviation of detrended error
* Drift → slope of smoothed error vs time

These metrics quantify GPS reliability during motion.

---

## How to Build

**Source ROS2 first:**

```bash
source /opt/ros/jazzy/setup.bash
```

Then build:

```bash
cd EECE5554
colcon build
source install/setup.bash
```

---

## Running the Analysis

Activate your Python environment if used:

```bash
source gnss_env/bin/activate
```

Run:

```bash
python analyze_gnss_bags.py \
  --bags <bag_paths> \
  --labels open occluded line200 \
  --topic /gps \
  --line_label line200
```

Outputs will be saved to:

```
lab2/analysis/out_gnss
```

---

## Key Findings

* GPS accuracy is significantly higher in open environments.
* Occlusions increase positional variance due to multipath interference.
* Straight-line analysis reveals measurable drift over time.
* HDOP correlates with observed positioning error.

---

## Technologies Used

* ROS2 Jazzy
* Python (NumPy, Pandas, Matplotlib)
* rosbag2_py
* UTM coordinate transformations

---

## Author

**Xinlei Li**
Master of Science in Electrical and Computer Engineering
Northeastern University

---

