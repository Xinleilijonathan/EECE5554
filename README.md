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
│           ├── gnss
│           │   ├── __init__.py
│           │   └── gnss_node.py
│           ├── launch
│           │   └── example_launch.py
│           ├── package.xml
│           ├── resource
│           ├── setup.cfg
│           └── setup.py
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

# Notes

This repository is structured to follow recommended ROS2 workspace practices and academic submission guidelines.

Future labs will be added as separate directories and documented accordingly.
