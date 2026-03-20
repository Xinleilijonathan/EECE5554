# EECE 5554 – Robotics Sensing and Navigation Labs

This repository contains all lab assignments completed for **EECE 5554 – Robotics Sensing and Navigation**.
Each lab is organized in its own directory and includes source code, analysis scripts, generated results, and a README describing the work performed.

---

# Repository Structure

```
EECE5554
├── lab1
│   ├── LASTNAME_turtle.png
│   ├── NortheasternUsername.txt
│   ├── README.md
│   └── src
│       ├── listener.py
│       └── talker.py
│
├── lab2
│   ├── analysis
│   │   └── analyze_gnss_bags.py
│   ├── data
│   │   └── your_rosbags   (not tracked in Git)
│   ├── README.md
│   └── src
│       └── gnss
│
├── lab3
│   ├── analysis 
│   │   ├── Xinlei_analysis_script.py
│   ├── data 
│   │   ├── occludedRTK.txt
│   │   ├── openRTK.txt
│   │   ├── walkingRTK.txt
│   ├── src
│   │   └── gnss_rtk
│   └── README.md
│
├── lab4
│   ├── analysis
│   │   └── analysis_script.py
│   ├── data
│   │   ├── stationary_5min      
│   │   └── freeform_2min        
│   ├── README.md
│   └── src
│       └── imu
│           ├── imu
│           │   ├── __init__.py
│           │   └── example_node.py
│           ├── launch
│           │   └── example_launch.py
│           ├── package.xml
│           ├── resource
│           │   └── imu
│           ├── setup.cfg
│           └── setup.py
│
└── README.md
```

**Note:** Build artifacts (`build/`, `install/`, `log/`) and rosbag files are excluded using `.gitignore`.

---

# Lab Descriptions

---

## Lab 1 – ROS2 Publisher & Subscriber Basics

The objective of this lab was to develop familiarity with:

* Linux command line operations
* Git repository workflow
* ROS2 nodes, topics, publishers, and subscribers
* Writing and modifying ROS2 Python nodes

### Contents

* **src/talker.py** — Modified ROS2 publisher node
* **src/listener.py** — Modified ROS2 subscriber node
* **LASTNAME_turtle.png** — Screenshot showing turtle simulation output
* **NortheasternUsername.txt** — Northeastern username file
* **README.md** — Detailed explanation of implementation

---

## Lab 2 – GNSS Driver Integration and Data Analysis

The goal of this lab was to integrate a GNSS driver within ROS2, collect positioning data, and evaluate localization performance under different environmental conditions.

### Key Learning Objectives

* Configure and build ROS2 Python packages
* Launch and run GNSS nodes
* Record sensor data using rosbag2
* Process GNSS datasets programmatically
* Analyze positioning accuracy, noise, and drift
* Generate professional plots for robotics evaluation

---

### Implemented Components

#### GNSS ROS2 Package

A fully structured ROS2 Python package named **`gnss`** was created, including:

* A functional GNSS node (`gnss_node.py`)
* Launch configuration for running the node
* Proper ROS2 packaging (`setup.py`, `package.xml`, `setup.cfg`)
* Colcon build compatibility

---

#### Data Collection

GNSS data was recorded in three environments:

* **Open Area** — minimal signal obstruction
* **Occluded Area** — reduced satellite visibility
* **Straight-Line Walk (200m)** — used for noise and drift analysis

---

#### Automated Analysis Pipeline

A custom Python script was developed to process rosbag data and generate evaluation plots.

**Generated Outputs:**

* UTM trajectory overlay
* Latitude vs Time
* Longitude vs Time
* Altitude vs Time
* HDOP vs Time
* Cross-track error analysis
* Noise histogram
* Drift estimation

These metrics help quantify GNSS reliability for robotic navigation tasks.

---

### Technical Skills Demonstrated

* ROS2 Jazzy ecosystem usage
* Sensor data parsing with `rosbag2_py`
* Numerical analysis with NumPy and Pandas
* Visualization with Matplotlib
* Coordinate-based error modeling (UTM cross-track error)
* Git version control with structured repositories

---

## Lab 3 – RTK GNSS Driver and Precision Positioning Analysis

The objective of this lab was to implement and evaluate a **Real-Time Kinematic (RTK) GNSS driver** in ROS2 and analyze the improvement in positioning accuracy compared with standalone GNSS.

RTK GNSS uses differential corrections from a reference station to significantly improve localization accuracy, enabling centimeter-level positioning under favorable conditions.

---

### Key Learning Objectives

* Implement a ROS2 driver for parsing RTK GNSS NMEA messages
* Extract positioning information from **GNGGA sentences**
* Convert latitude/longitude measurements into **UTM coordinates**
* Evaluate positioning accuracy using statistical metrics
* Compare **Standalone GNSS vs RTK GNSS performance**
* Analyze positioning stability in both open and occluded environments

---

### Implemented Components

#### RTK GNSS ROS2 Driver

A ROS2 Python package named **`gnss_rtk`** was developed to interface with the RTK GNSS receiver.

The driver:

* Reads NMEA data from a serial interface
* Parses **GNGGA messages**
* Extracts the following fields:

  * UTC timestamp
  * Latitude / Longitude
  * Altitude
  * Fix quality
  * Number of satellites
  * HDOP

The parsed data can then be published as ROS messages for downstream robotics applications.

---

#### Data Collection

Three datasets were collected for analysis:

* **Open-sky stationary measurements** — minimal satellite obstruction
* **Occluded stationary measurements** — reduced satellite visibility and multipath effects
* **Walking trajectory dataset** — used to evaluate trajectory accuracy

These datasets were processed to evaluate GNSS accuracy under different environmental conditions.

---

#### Data Analysis

Custom Python analysis scripts were used to generate quantitative evaluation metrics and visualization plots.

The analysis includes:

* Stationary **Northing vs Easting scatter plots**
* **Horizontal error histograms**
* **Altitude stability analysis**
* **Walking trajectory evaluation**
* **Line-fit trajectory error estimation**
* **RMSE comparison between standalone GNSS and RTK GNSS**

---

### Results Summary

The results demonstrate the significant improvement provided by RTK corrections.

Key observations include:

* **Open-sky RTK positioning achieved sub-meter accuracy**
* **Occluded environments produced larger error due to multipath and satellite blockage**
* **RTK GNSS significantly outperformed standalone GNSS in all evaluated scenarios**
* Walking trajectory analysis showed that RTK maintained a highly linear path with minimal lateral deviation.

These results confirm the benefits of RTK GNSS for robotics navigation tasks requiring high positional accuracy.

---

### Technical Skills Demonstrated

* ROS2 driver development
* NMEA GNSS message parsing
* Coordinate system transformation (Latitude/Longitude → UTM)
* Statistical error analysis
* Visualization of positioning datasets
* Performance comparison between GNSS localization methods

---

## Lab 4 – IMU Data Analysis and Sensor Noise Characterization

The objective of this lab was to analyze **Inertial Measurement Unit (IMU)** data and understand sensor noise characteristics and motion behavior.

### Key Learning Objectives

* Work with IMU data (gyroscope and accelerometer) in ROS2  
* Record sensor data using rosbag2  
* Analyze sensor noise using **Allan deviation**  
* Estimate IMU noise parameters (ARW, bias instability)  
* Perform orientation estimation (roll, pitch, yaw)  
* Evaluate IMU behavior during dynamic motion  

---

### Implemented Components

#### IMU ROS2 Package

A ROS2 Python package named **`imu`** was developed to publish IMU measurements.

The package includes:

* IMU node (`example_node.py`)  
* Launch file for running the node  
* Standard ROS2 package structure  

---

#### Data Collection

Two types of datasets were collected:

* **Stationary data** — used for noise and Allan deviation analysis  
* **Free-form motion data** — used to observe dynamic IMU behavior  

---

#### Data Analysis

Custom Python scripts were developed to analyze IMU data.

The analysis includes:

* Gyroscope (ωx, ωy, ωz) visualization  
* Accelerometer (ax, ay, az) visualization  
* Orientation estimation (roll, pitch, yaw)  
* Allan deviation computation  
* Noise parameter estimation  

---

### Results Summary

* Gyroscope signals show clear spikes during rotational motion  
* Accelerometer captures both gravity and dynamic acceleration  
* Orientation estimation is smooth but exhibits **drift over time**, especially in yaw  
* Allan deviation analysis shows dominant **angle random walk (ARW)** behavior  
* Bias instability and rate random walk are not clearly observable in the dataset  

---

### Technical Skills Demonstrated

* IMU sensor data processing  
* ROS2 sensor integration  
* Time-series signal analysis  
* Allan variance and noise modeling  
* Orientation estimation from IMU data  
* Data visualization and interpretation

---

# Notes

This repository is structured to follow recommended ROS2 workspace practices and academic submission guidelines.

Future labs will be added as separate directories and documented accordingly.
