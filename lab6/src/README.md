# Lab 6 — ROS 2 Source Packages

## Packages

| Package | Type | Node | Topic | Rate | Description |
|---|---|---|---|---|---|
| `imu_msgs` | ament_cmake | — | — | — | Custom `Vectornav` message definition |
| `imu` | ament_python | `imu_node` | `/imu` (Vectornav) | 40 Hz | VectorNav VN-100 IMU driver |
| `gnss` | ament_python | `gnss_node` | `/gps` (Customgps) | 5 Hz | Standalone GPS puck driver |

Build order: `imu_msgs` → `imu`, `gnss`

## Package Details

### `imu_msgs`

Custom message package required by the `imu` driver.

**Message: `Vectornav`**

```
std_msgs/Header header
sensor_msgs/Imu imu              # orientation (quat), angular velocity, linear accel
sensor_msgs/MagneticField mag    # magnetometer XYZ
string raw_str                   # raw VNYMR sentence for debugging
```

### `imu`

Driver for the VectorNav VN-100 IMU over USB serial.

- Parses `$VNYMR` NMEA-style sentences (yaw, pitch, roll, mag XYZ, accel XYZ, gyro XYZ)
- Publishes orientation as quaternion (ROS convention)
- Uses `self.get_clock().now()` for epoch-aligned timestamps
- Default baud rate: 115200

**Launch:**
```bash
ros2 launch imu imu_driver.launch.py port:=/dev/ttyUSB0
```

**Entry points:**
- `imu_node` → `imu.driver:main`
- `example_node` → `imu.example_node:main`

### `gnss`

Driver for a standalone USB GPS puck.

- Parses NMEA `$GPGGA` sentences (latitude, longitude, altitude, fix quality, HDOP)
- Converts geodetic coordinates to UTM (easting, northing) using the `utm` library
- Uses `self.get_clock().now()` for epoch-aligned timestamps
- Default baud rate: 4800

**Launch:**
```bash
ros2 launch gnss gnss_driver.launch.py port:=/dev/ttyUSB1
```

**Entry points:**
- `gnss_node` → `gnss.gnss_node:main`

### `launch/lab6_launch.py`

Unified launch file that brings up both IMU and GPS driver nodes simultaneously.

```bash
# Default ports
ros2 launch lab6_launch.py

# Custom ports
ros2 launch lab6_launch.py imu_port:=/dev/ttyUSB0 gps_port:=/dev/ttyUSB1
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `imu_port` | `/dev/ttyUSB0` | Serial port for VN-100 IMU |
| `gps_port` | `/dev/ttyUSB1` | Serial port for GPS puck |
| `imu_baud` | `115200` | IMU baud rate |
| `gps_baud` | `4800` | GPS baud rate |

## Building All Packages

```bash
cd ~/ros2_ws
colcon build --packages-select imu_msgs gnss imu
source install/setup.bash
```

## Verifying Topics

```bash
ros2 topic echo /imu
ros2 topic echo /gps
ros2 topic hz /imu   # expect ~40 Hz
ros2 topic hz /gps   # expect ~5 Hz
```
