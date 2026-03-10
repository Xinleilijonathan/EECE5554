# EECE5554 Lab 3 — RTK GNSS Driver and Analysis

Implementation and analysis of a ROS2 RTK GNSS driver and evaluation of RTK positioning performance compared with standalone GNSS.

This project includes:

- A ROS2 driver for parsing RTK GNSS NMEA messages
- Conversion of latitude/longitude to UTM coordinates
- Analysis scripts for evaluating positioning accuracy
- Visualization of stationary and walking GNSS datasets

---

# Project Overview

RTK (Real-Time Kinematic) GNSS improves positioning accuracy by using differential corrections from a base station.  
This lab evaluates the performance difference between:

- **Standalone GNSS**
- **RTK GNSS**

Experiments include:

- Stationary positioning (open sky)
- Stationary positioning (occluded environment)
- Walking trajectory analysis

Performance metrics include:

- Horizontal positioning error
- Altitude stability
- Trajectory consistency
- RMSE analysis

---

# Repository Structure

lab3
├── analysis # Python scripts for GNSS data analysis
│ ├── Xinlei_analysis_script.py
│ ├── q2_stationary_xy_rel_open_occluded.png
│ ├── RTK_histogram.png
│ ├── RTK_altitude.png
│ ├── RTK_walking_NE.png
│ └── trajectory_utm_overlay.png
│
├── data # rosbag datasets used for testing
│
├── src
│ └── gnss_rtk # ROS2 package implementing RTK GNSS driver
│ ├── launch
│ ├── msg
│ ├── scripts
│ └── package.xml
│
└── README.md

---

# Dataset

RTK data is processed using a **sensor emulator** provided by the course staff.

The emulator streams **NMEA messages** to simulate GNSS hardware output.

The driver parses the following information from `$GNGGA` messages:

- Latitude
- Longitude
- Altitude
- UTM Easting
- UTM Northing
- Fix Quality
- HDOP
- Timestamp

---

# Installation

This project requires **ROS2 (Jazzy)**.

Install dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop

Clone the repository into a ROS2 workspace.

Build Instructions

Navigate to the workspace root:

cd lab3

Build the ROS2 package:

colcon build

Source the workspace:

source install/setup.bash
Running the RTK GNSS Driver

Launch the RTK GNSS driver:

ros2 launch gnss_rtk rtk_driver_launch.py

The node will start reading NMEA messages from the emulator and publishing GNSS data.

Output

The driver publishes RTK GNSS information including:

Latitude
Longitude
Altitude
UTM Easting
UTM Northing
Fix Quality
HDOP

These values can be used for navigation and localization analysis.

Data Analysis

The analysis scripts compute GNSS positioning accuracy using:

Centroid-based error (stationary datasets)

Best-fit line error (walking datasets)

Root Mean Square Error (RMSE)

Example analysis:

python Xinlei_analysis_script.py

The script generates:

Stationary position scatter plots

Error histograms

Altitude vs time plots

Walking trajectory plots

Example Results
Stationary Position Error

Open sky conditions show tight clustering around the centroid, indicating high RTK accuracy.

Error Distribution

RTK measurements under open sky remain within sub-meter accuracy, while occluded environments introduce larger positioning error.

Walking Trajectory

RTK GNSS maintains high trajectory consistency during walking experiments with very small lateral deviation.

Example visualizations:

Horizontal position scatter

Error histogram

Altitude stability plot

Walking trajectory

Author

Xinlei Li
MS Electrical and Computer Engineering
Northeastern University
