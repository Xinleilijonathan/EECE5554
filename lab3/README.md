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

---
```
lab3
├── analysis 
│ ├── Xinlei_analysis_script.py
├── data 
│ ├── occludedRTK.txt
│ ├── openRTK.txt
│ ├── walkingRTK.txt
├── src
│     ├── gnss_rtk
│     │   ├── gnss_rtk
│     │   │   ├── __init__.py
│     │   │   └── rtk_driver.py
│     │   ├── launch
│     │   │   └── rtk_driver_launch.py
│     │   ├── package.xml
│     │   ├── resource
│     │   │   └── gnss_rtk
│     │   ├── setup.cfg
│     │   └── setup.py
│     └── README.md
└── README.md
```
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

## Installation

This project requires **ROS2 Jazzy**.

Install ROS2 dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install python3-colcon-common-extensions
```

## Create a ROS2 Workspace

Create a ROS2 workspace and clone the repository:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Xinleijonathan/EECE5554.git
```

Navigate to the workspace root:

```bash
cd ~/ros2_ws
```

## Build Instructions

Build the ROS2 package:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

(Optional) Automatically source the workspace when opening a terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Running the RTK GNSS Driver

Launch the RTK GNSS driver:

```bash
ros2 launch gnss_rtk rtk_driver_launch.py
```

The node will start reading **NMEA messages from the GNSS emulator** and publishing GNSS data.

## Output

The driver publishes RTK GNSS information including:

- Latitude
- Longitude
- Altitude
- UTM Easting
- UTM Northing
- Fix Quality
- HDOP
- Timestamp

These values can be used for navigation and localization analysis.

## Data Analysis

The analysis scripts compute GNSS positioning accuracy using:

- Centroid-based error (stationary datasets)
- Best-fit line error (walking datasets)
- Root Mean Square Error (RMSE)

Run the analysis script:

```bash
python Xinlei_analysis_script.py
```

The script generates:

- Stationary position scatter plots
- Error histograms
- Altitude vs time plots
- Walking trajectory plots

## Example Results

### Stationary Position Error

Open-sky conditions show tight clustering around the centroid, indicating high RTK positioning accuracy.

### Error Distribution

RTK measurements under open-sky conditions remain within **sub-meter accuracy**, while occluded environments introduce larger positioning error.

### Walking Trajectory

RTK GNSS maintains high trajectory consistency during walking experiments with very small lateral deviation.

Example visualizations include:

- Horizontal position scatter
- Error histogram
- Altitude stability plot
- Walking trajectory

## Author

Xinlei Li  
MS Electrical and Computer Engineering  
Northeastern University



