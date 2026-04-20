#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lab6_analysis.py

Lab 6 NUance multi-sensor navigation analysis.
Produces Figs 0-10 and prints Q5/Q7 statistics to stdout.

Bag paths:
  BAG_CALIB -> calibration (circles)   : data_parking_lot
  BAG_DRIVE -> city driving             : data_long_drive_loops

Before running confirm topic names with:
    ros2 bag info /media/jonathancyriane/SSD/GroupA/data_parking_lot
"""

from pathlib import Path
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any

import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

from scipy.integrate import cumulative_trapezoid
from scipy.signal import butter, sosfiltfilt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


# =============================================================================
# CONFIG
# =============================================================================

# External IMU+GPS bags (your VN-100 + GPS puck, on your laptop)
BAG_CALIB_EXT = Path("/home/ur12e/Desktop/EECE5554/lab6/imu_and_gps_in_a_circle")
BAG_DRIVE_EXT = Path("/home/ur12e/Desktop/EECE5554/lab6/Around_the_city_imu_and_gps")

# NUance vehicle bags (LiDAR + camera from GroupA SSD)
BAG_CALIB_VEH = Path("/media/ur12e/SSD/GroupA/data_drive_in_a_circle_vehicle")
BAG_DRIVE_VEH = Path("/media/ur12e/SSD/GroupA/data_drive_around_city_vehicle")

OUTPUT_DIR = Path.home() / "EECE5554" / "lab6" / "analysis" / "figures"

TOPICS = {
    "imu":    "/imu",
    "gps":    "/gps",
    "lidar":  "/front_lidar/points",   # vehicle bag topic name
    "camera": "/front_camera/image_rect",  # vehicle bag topic name
    "radar":  None,
}

REF_LAT = None
REF_LON = None
SAVE_DPI = 180

COMPLEMENTARY_ALPHA = 0.5
FILTER_ORDER = 2
LPF_CUTOFF_HZ = 0.5
HPF_CUTOFF_HZ = 0.05

LIDAR_VOXEL_SIZE = 0.5
LIDAR_MAX_POINTS = 25000
LIDAR_MAX_RANGE_METERS = 80.0
LIDAR_MIN_RANGE_METERS = 1.0
ICP_MAX_CORRESPONDENCE = 1.5

NUM_CAMERA_FRAMES = 4
LIDAR_SNAPSHOT_INDICES = [10, 50]
ENABLE_RADAR = False


# =============================================================================
# DATA CONTAINERS
# =============================================================================

@dataclass
class IMUSample:
    t: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: Optional[float] = None
    my: Optional[float] = None
    mz: Optional[float] = None
    heading_imu: Optional[float] = None


@dataclass
class GPSSample:
    t: float
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    speed: Optional[float] = None
    e: Optional[float] = None
    n: Optional[float] = None


@dataclass
class PointCloudSample:
    t: float
    xyz: np.ndarray


@dataclass
class CameraSample:
    t: float
    image: np.ndarray


# =============================================================================
# HELPERS
# =============================================================================

def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_angle(angle: np.ndarray) -> np.ndarray:
    return np.unwrap(angle)


def butter_filter(data: np.ndarray, cutoff_hz: float, sample_hz: float,
                  btype: str, order: int) -> np.ndarray:
    nyquist = 0.5 * sample_hz
    if cutoff_hz <= 0 or cutoff_hz >= nyquist:
        raise ValueError(f"Cutoff {cutoff_hz} must be between 0 and Nyquist {nyquist}")
    sos = butter(order, cutoff_hz / nyquist, btype=btype, output="sos")
    return sosfiltfilt(sos, data)


def estimate_sample_rate(t: np.ndarray) -> float:
    dt = np.diff(t)
    dt = dt[dt > 0]
    if len(dt) == 0:
        return 1.0
    return 1.0 / np.median(dt)


def save_figure(fig: plt.Figure, filename: str) -> None:
    ensure_dir(OUTPUT_DIR)
    out_path = OUTPUT_DIR / filename
    fig.tight_layout()
    fig.savefig(out_path, dpi=SAVE_DPI, bbox_inches="tight")
    print(f"[Saved] {out_path}")


def geodetic_to_local_xy(lat: np.ndarray, lon: np.ndarray,
                         ref_lat: float, ref_lon: float) -> Tuple[np.ndarray, np.ndarray]:
    R = 6378137.0
    dlat = np.deg2rad(lat) - np.deg2rad(ref_lat)
    dlon = np.deg2rad(lon) - np.deg2rad(ref_lon)
    north = R * dlat
    east  = R * np.cos(np.deg2rad(ref_lat)) * dlon
    return east, north


def get_attr(obj: Any, path: List[str], default=None):
    current = obj
    for name in path:
        if not hasattr(current, name):
            return default
        current = getattr(current, name)
    return current


def first_numeric_value(obj: Any, candidate_paths: List[List[str]]) -> Optional[float]:
    for path in candidate_paths:
        value = get_attr(obj, path, None)
        if value is None:
            continue
        try:
            return float(value)
        except Exception:
            continue
    return None


def safe_float_or_zero(value: Optional[float]) -> float:
    return 0.0 if value is None else float(value)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> Optional[float]:
    if qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0:
        return None
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# =============================================================================
# ROS BAG READING  (fix: initialise `out` before the loop)
# =============================================================================

def read_bag_messages(bag_path: Path,
                      wanted_topics: List[str]) -> Dict[str, List[Tuple[float, Any]]]:
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")

    out: Dict[str, List[Tuple[float, Any]]] = {topic: [] for topic in wanted_topics}
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in wanted_topics]
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            try:
                msg = reader.deserialize(rawdata, connection.msgtype)
            except Exception:
                continue
            t = timestamp_ns * 1e-9
            if connection.topic in out:
                out[connection.topic].append((t, msg))

    return out


# =============================================================================
# MESSAGE ADAPTERS
# =============================================================================

def extract_imu_sample(msg: Any, t: float) -> Optional[IMUSample]:
    ax = first_numeric_value(msg, [
        ["linear_acceleration", "x"], ["accel", "x"],
        ["accelerometer", "x"], ["imu", "linear_acceleration", "x"],
    ])
    ay = first_numeric_value(msg, [
        ["linear_acceleration", "y"], ["accel", "y"],
        ["accelerometer", "y"], ["imu", "linear_acceleration", "y"],
    ])
    az = first_numeric_value(msg, [
        ["linear_acceleration", "z"], ["accel", "z"],
        ["accelerometer", "z"], ["imu", "linear_acceleration", "z"],
    ])
    gx = first_numeric_value(msg, [
        ["angular_velocity", "x"], ["gyro", "x"],
        ["gyroscope", "x"], ["imu", "angular_velocity", "x"],
    ])
    gy = first_numeric_value(msg, [
        ["angular_velocity", "y"], ["gyro", "y"],
        ["gyroscope", "y"], ["imu", "angular_velocity", "y"],
    ])
    gz = first_numeric_value(msg, [
        ["angular_velocity", "z"], ["gyro", "z"],
        ["gyroscope", "z"], ["imu", "angular_velocity", "z"],
    ])
    mx = first_numeric_value(msg, [
        ["magnetic_field", "x"], ["mag", "x"], ["magnetometer", "x"],
        ["field", "x"], ["imu", "magnetic_field", "x"],
        ["mag_field", "magnetic_field", "x"],
    ])
    my = first_numeric_value(msg, [
        ["magnetic_field", "y"], ["mag", "y"], ["magnetometer", "y"],
        ["field", "y"], ["imu", "magnetic_field", "y"],
        ["mag_field", "magnetic_field", "y"],
    ])
    mz = first_numeric_value(msg, [
        ["magnetic_field", "z"], ["mag", "z"], ["magnetometer", "z"],
        ["field", "z"], ["imu", "magnetic_field", "z"],
        ["mag_field", "magnetic_field", "z"],
    ])
    qx = first_numeric_value(msg, [
        ["orientation", "x"], ["quat", "x"], ["quaternion", "x"],
        ["imu", "orientation", "x"],
    ])
    qy = first_numeric_value(msg, [
        ["orientation", "y"], ["quat", "y"], ["quaternion", "y"],
        ["imu", "orientation", "y"],
    ])
    qz = first_numeric_value(msg, [
        ["orientation", "z"], ["quat", "z"], ["quaternion", "z"],
        ["imu", "orientation", "z"],
    ])
    qw = first_numeric_value(msg, [
        ["orientation", "w"], ["quat", "w"], ["quaternion", "w"],
        ["imu", "orientation", "w"],
    ])

    heading_imu = None
    if qx is not None and qy is not None and qz is not None and qw is not None:
        heading_imu = quaternion_to_yaw(qx, qy, qz, qw)
    if heading_imu is None and mx is not None and my is not None:
        heading_imu = math.atan2(my, mx)

    if ax is None or ay is None or az is None or gx is None or gy is None or gz is None:
        return None

    return IMUSample(
        t=t,
        ax=safe_float_or_zero(ax), ay=safe_float_or_zero(ay), az=safe_float_or_zero(az),
        gx=safe_float_or_zero(gx), gy=safe_float_or_zero(gy), gz=safe_float_or_zero(gz),
        mx=None if mx is None else float(mx),
        my=None if my is None else float(my),
        mz=None if mz is None else float(mz),
        heading_imu=None if heading_imu is None else float(heading_imu),
    )


def extract_gps_sample(msg: Any, t: float) -> Optional[GPSSample]:
    lat   = first_numeric_value(msg, [["latitude"],  ["lat"]])
    lon   = first_numeric_value(msg, [["longitude"], ["lon"], ["lng"]])
    alt   = first_numeric_value(msg, [["altitude"],  ["alt"]])
    speed = first_numeric_value(msg, [["speed"], ["velocity"], ["ground_speed"]])
    e     = first_numeric_value(msg, [["utm_easting"],  ["easting"], ["x"], ["enu", "x"]])
    n     = first_numeric_value(msg, [["utm_northing"], ["northing"], ["y"], ["enu", "y"]])

    if lat is None and lon is None and e is None and n is None:
        return None

    return GPSSample(t=t, lat=lat, lon=lon, alt=alt, speed=speed, e=e, n=n)


def pointcloud2_to_xyz(msg: Any) -> np.ndarray:
    n_points = msg.width * msg.height
    point_step = msg.point_step
    fields = {field.name: field.offset for field in msg.fields}
    if "x" not in fields or "y" not in fields or "z" not in fields:
        raise ValueError("PointCloud2 missing x/y/z fields.")
    data = np.frombuffer(msg.data, dtype=np.uint8)
    expected = n_points * point_step
    if len(data) < expected:
        raise ValueError(f"buffer is smaller than expected: got {len(data)}, need {expected}")
    # Reshape to (n_points, point_step) so each row is one point's raw bytes
    data_2d = data[:expected].reshape(n_points, point_step)
    xyz = np.empty((n_points, 3), dtype=np.float32)
    for i, key in enumerate(["x", "y", "z"]):
        off = fields[key]
        xyz[:, i] = data_2d[:, off:off + 4].copy().view(np.float32).reshape(-1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def extract_camera_image(msg: Any) -> np.ndarray:
    h, w = msg.height, msg.width
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding == "rgb8":
        return cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return data.reshape((h, w, 3))
    if encoding == "mono8":
        return cv2.cvtColor(data.reshape((h, w)), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


# =============================================================================
# DATA LOADING
# =============================================================================

def load_imu_series(bag_path: Path, imu_topic: str) -> List[IMUSample]:
    data = read_bag_messages(bag_path, [imu_topic])[imu_topic]
    return [s for t, msg in data for s in [extract_imu_sample(msg, t)] if s is not None]


def load_gps_series(bag_path: Path, gps_topic: str) -> List[GPSSample]:
    data = read_bag_messages(bag_path, [gps_topic])[gps_topic]
    return [s for t, msg in data for s in [extract_gps_sample(msg, t)] if s is not None]


def load_lidar_series(bag_path: Path, lidar_topic: str,
                      stride: int = 1) -> List[PointCloudSample]:
    """Stream LiDAR frames one at a time to avoid loading the full bag into RAM.

    stride: keep every Nth frame (1 = all frames).
    """
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    out = []
    frame_idx = 0
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == lidar_topic]
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            if frame_idx % stride == 0:
                t = timestamp_ns * 1e-9
                try:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    out.append(PointCloudSample(t=t, xyz=pointcloud2_to_xyz(msg)))
                except Exception as exc:
                    print(f"[WARN] LiDAR parse failed at t={t:.3f}: {exc}")
            frame_idx += 1
    return out


def load_camera_series(bag_path: Path, camera_topic: str,
                       max_frames: Optional[int] = None,
                       stride: int = 1) -> List[CameraSample]:
    """Stream camera frames one at a time to avoid loading the full bag into RAM.

    stride: decode only every Nth frame (preserves temporal spread across the drive).
    max_frames: hard cap on decoded frames after stride filtering.
    """
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    out = []
    frame_idx = 0
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == camera_topic]
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            if max_frames is not None and len(out) >= max_frames:
                break
            if frame_idx % stride == 0:
                t = timestamp_ns * 1e-9
                try:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    out.append(CameraSample(t=t, image=extract_camera_image(msg)))
                except Exception as exc:
                    print(f"[WARN] Camera parse failed at t={t:.3f}: {exc}")
            frame_idx += 1
    return out


# =============================================================================
# MAGNETOMETER CALIBRATION  (hard + soft iron via least-squares ellipse fit)
# =============================================================================

def fit_mag_calibration(mx: np.ndarray, my: np.ndarray):
    """
    Fit a general ellipse to (mx, my) using least squares.
    Data is normalised to unit scale before fitting to avoid numerical issues
    with very small magnetometer values (e.g. Tesla-range ~1e-5).

    Returns center, semi_axes, eigenvectors — all in original units.
    """
    x, y = mx.astype(float), my.astype(float)

    # Normalise so the fitting problem is well-conditioned
    x_scale = np.std(x) if np.std(x) > 0 else 1.0
    y_scale = np.std(y) if np.std(y) > 0 else 1.0
    xn, yn = x / x_scale, y / y_scale

    D = np.column_stack([xn**2, xn * yn, yn**2, xn, yn])
    params, _, _, _ = np.linalg.lstsq(D, np.ones(len(xn)), rcond=None)
    A, B, C, D_c, E = params
    F = -1.0

    # Center in normalised space
    M = np.array([[2*A, B], [B, 2*C]])
    rhs = np.array([-D_c, -E])
    try:
        center_n = np.linalg.solve(M, rhs)
    except np.linalg.LinAlgError:
        center_n = np.array([0.0, 0.0])

    # Convert center back to original units
    center = np.array([center_n[0] * x_scale, center_n[1] * y_scale])

    # Value of conic at normalised center
    Fc = (A * center_n[0]**2 + B * center_n[0] * center_n[1]
          + C * center_n[1]**2 + D_c * center_n[0] + E * center_n[1] + F)
    if abs(Fc) < 1e-30:
        Fc = -1.0

    # Shape matrix in normalised space → eigendecomposition gives axes
    M_shape = np.array([[A, B / 2], [B / 2, C]]) / (-Fc)
    eigenvalues, eigenvectors = np.linalg.eigh(M_shape)
    eigenvalues = np.maximum(eigenvalues, 1e-30)

    # Semi-axes in normalised space → convert to original units
    semi_axes_n = 1.0 / np.sqrt(eigenvalues)
    # eigenvectors[:,0] aligns with x, [:,1] with y in normalised space
    semi_axes = semi_axes_n * np.array([x_scale, y_scale])

    return center, semi_axes, eigenvectors


def apply_mag_calibration(mx: np.ndarray, my: np.ndarray,
                          center: np.ndarray, semi_axes: np.ndarray,
                          eigenvectors: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Apply hard+soft iron correction: centre → rotate → normalise → rotate back."""
    xy = np.column_stack([mx - center[0], my - center[1]])
    xy_rot = xy @ eigenvectors
    avg_axis = np.mean(semi_axes)
    xy_scaled = xy_rot / semi_axes * avg_axis
    xy_out = xy_scaled @ eigenvectors.T
    return xy_out[:, 0], xy_out[:, 1]


def yaw_from_mag(mx: np.ndarray, my: np.ndarray) -> np.ndarray:
    return wrap_to_pi(np.arctan2(my, mx))


# =============================================================================
# NUMERIC ANALYSIS
# =============================================================================

def series_from_imu(samples: List[IMUSample]) -> Dict[str, np.ndarray]:
    t = np.array([s.t for s in samples])
    t = t - t[0]
    return {
        "t":           t,
        "ax":          np.array([s.ax for s in samples]),
        "ay":          np.array([s.ay for s in samples]),
        "az":          np.array([s.az for s in samples]),
        "gx":          np.array([s.gx for s in samples]),
        "gy":          np.array([s.gy for s in samples]),
        "gz":          np.array([s.gz for s in samples]),
        "mx":          np.array([np.nan if s.mx is None else s.mx for s in samples]),
        "my":          np.array([np.nan if s.my is None else s.my for s in samples]),
        "mz":          np.array([np.nan if s.mz is None else s.mz for s in samples]),
        "heading_imu": np.array([np.nan if s.heading_imu is None else s.heading_imu
                                 for s in samples]),
    }


def series_from_gps(samples: List[GPSSample]) -> Dict[str, np.ndarray]:
    t = np.array([s.t for s in samples])
    t = t - t[0]

    lat   = np.array([np.nan if s.lat   is None else s.lat   for s in samples])
    lon   = np.array([np.nan if s.lon   is None else s.lon   for s in samples])
    speed = np.array([np.nan if s.speed is None else s.speed for s in samples])
    e     = np.array([np.nan if s.e     is None else s.e     for s in samples])
    n     = np.array([np.nan if s.n     is None else s.n     for s in samples])

    if np.all(np.isnan(e)) or np.all(np.isnan(n)):
        valid = np.isfinite(lat) & np.isfinite(lon)
        if np.any(valid):
            ref_lat = lat[valid][0] if REF_LAT is None else REF_LAT
            ref_lon = lon[valid][0] if REF_LON is None else REF_LON
            e2, n2 = geodetic_to_local_xy(lat[valid], lon[valid], ref_lat, ref_lon)
            e = np.full_like(lat, np.nan)
            n = np.full_like(lon, np.nan)
            e[valid] = e2
            n[valid] = n2

    return {"t": t, "lat": lat, "lon": lon, "speed": speed, "e": e, "n": n}


def integrate_gyro_yaw(t: np.ndarray, gz: np.ndarray) -> np.ndarray:
    return wrap_to_pi(cumulative_trapezoid(gz, t, initial=0.0))


def estimate_forward_velocity_from_accel(
        t: np.ndarray, ax: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    raw_v = cumulative_trapezoid(ax, t, initial=0.0)
    n_bias = max(10, min(len(ax) // 10, int(np.sum(t <= 5.0))))
    bias = np.mean(ax[:n_bias])
    adj_v = cumulative_trapezoid(ax - bias, t, initial=0.0)
    return raw_v, adj_v


def estimate_gps_velocity(t: np.ndarray, e: np.ndarray, n: np.ndarray,
                          speed_direct: np.ndarray) -> np.ndarray:
    if np.any(np.isfinite(speed_direct)):
        return speed_direct
    return np.sqrt(np.gradient(e, t, edge_order=1)**2 + np.gradient(n, t, edge_order=1)**2)


def align_imu_to_gps_heading(imu_e: np.ndarray, imu_n: np.ndarray,
                              gps_e: np.ndarray, gps_n: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Rotate IMU trajectory so its initial heading matches GPS initial heading."""
    n_pts = min(max(5, len(gps_e) // 20), 20)

    gps_head = np.arctan2(gps_n[n_pts] - gps_n[0], gps_e[n_pts] - gps_e[0])
    imu_head = np.arctan2(imu_n[n_pts] - imu_n[0], imu_e[n_pts] - imu_e[0])
    delta = gps_head - imu_head

    c, s = np.cos(delta), np.sin(delta)
    return imu_e * c - imu_n * s, imu_e * s + imu_n * c


def build_imu_trajectory(t: np.ndarray, v_forward: np.ndarray,
                         yaw: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    ve = v_forward * np.cos(yaw)
    vn = v_forward * np.sin(yaw)
    xe = cumulative_trapezoid(ve, t, initial=0.0)
    xn = cumulative_trapezoid(vn, t, initial=0.0)
    return xe, xn


def compute_centripetal(t: np.ndarray, gz: np.ndarray,
                        v_forward: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return omega*Vx (predicted ay) and observed ay for Q5."""
    omega_vx = gz * v_forward
    return omega_vx


def compute_gps_imu_agreement(gps_e: np.ndarray, gps_n: np.ndarray,
                               gps_t: np.ndarray,
                               imu_e: np.ndarray, imu_n: np.ndarray,
                               imu_t: np.ndarray,
                               threshold_m: float = 2.0) -> Tuple[float, float]:
    """
    Return (max_duration_s, end_time_s) of longest continuous period where
    interpolated GPS-IMU distance stays within threshold_m.
    """
    # Interpolate IMU onto GPS times
    valid_g = np.isfinite(gps_e) & np.isfinite(gps_n)
    tg = gps_t[valid_g]
    eg = gps_e[valid_g]
    ng = gps_n[valid_g]

    overlap_mask = (tg >= imu_t[0]) & (tg <= imu_t[-1])
    tg = tg[overlap_mask]
    eg = eg[overlap_mask]
    ng = ng[overlap_mask]

    if len(tg) < 2:
        return 0.0, 0.0

    imu_e_interp = np.interp(tg, imu_t, imu_e)
    imu_n_interp = np.interp(tg, imu_t, imu_n)

    # Align starting point
    imu_e_interp = imu_e_interp - imu_e_interp[0] + eg[0]
    imu_n_interp = imu_n_interp - imu_n_interp[0] + ng[0]

    dist = np.sqrt((imu_e_interp - eg)**2 + (imu_n_interp - ng)**2)
    within = dist < threshold_m

    max_dur, cur_start, best_end = 0.0, tg[0], tg[0]
    in_streak = False
    for i, w in enumerate(within):
        if w and not in_streak:
            cur_start = tg[i]
            in_streak = True
        elif not w and in_streak:
            dur = tg[i - 1] - cur_start
            if dur > max_dur:
                max_dur = dur
                best_end = tg[i - 1]
            in_streak = False
    if in_streak:
        dur = tg[-1] - cur_start
        if dur > max_dur:
            max_dur = dur
            best_end = tg[-1]

    return max_dur, best_end


# =============================================================================
# LIDAR PROCESSING
# =============================================================================

def filter_downsample_xyz(xyz: np.ndarray) -> np.ndarray:
    ranges = np.linalg.norm(xyz[:, :3], axis=1)
    mask = (ranges >= LIDAR_MIN_RANGE_METERS) & (ranges <= LIDAR_MAX_RANGE_METERS)
    xyz = xyz[mask]
    if len(xyz) > LIDAR_MAX_POINTS:
        idx = np.random.choice(len(xyz), size=LIDAR_MAX_POINTS, replace=False)
        xyz = xyz[idx]
    return xyz


def xyz_to_o3d(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    return pc.voxel_down_sample(voxel_size=LIDAR_VOXEL_SIZE)


def estimate_lidar_odometry(lidar_samples: List[PointCloudSample]) -> Tuple[np.ndarray, np.ndarray]:
    if len(lidar_samples) < 2:
        return np.array([0.0]), np.array([0.0])

    x_positions, y_positions = [0.0], [0.0]
    T_world = np.eye(4)

    prev_xyz = filter_downsample_xyz(lidar_samples[0].xyz)
    prev_pc = xyz_to_o3d(prev_xyz)
    prev_pc.estimate_normals()

    for i in range(1, len(lidar_samples)):
        curr_xyz = filter_downsample_xyz(lidar_samples[i].xyz)
        curr_pc = xyz_to_o3d(curr_xyz)
        curr_pc.estimate_normals()

        reg = o3d.pipelines.registration.registration_icp(
            curr_pc, prev_pc, ICP_MAX_CORRESPONDENCE, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )

        T_world = T_world @ np.linalg.inv(reg.transformation)
        x_positions.append(T_world[0, 3])
        y_positions.append(T_world[1, 3])
        prev_pc = curr_pc

    return np.array(x_positions), np.array(y_positions)


def print_lidar_stats(lidar_samples: List[PointCloudSample]) -> None:
    if len(lidar_samples) < 2:
        return
    durations = np.diff([s.t for s in lidar_samples])
    fps = 1.0 / np.median(durations[durations > 0])
    n_points = [len(s.xyz) for s in lidar_samples[:20]]
    print(f"[LiDAR] Frame rate: {fps:.1f} Hz | "
          f"Points/scan (median): {int(np.median(n_points))} | "
          f"Frames total: {len(lidar_samples)}")


# =============================================================================
# PLOTTING
# =============================================================================

def plot_fig0_mag_scatter(mx_raw, my_raw, mx_cal, my_cal):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].scatter(mx_raw, my_raw, s=3, alpha=0.5)
    axes[0].set_title("Fig. 0a — Magnetometer Before Calibration")
    axes[0].set_xlabel("Mag X")
    axes[0].set_ylabel("Mag Y")
    axes[0].axis("equal")
    axes[0].grid(True)

    axes[1].scatter(mx_cal, my_cal, s=3, alpha=0.5, color="orange")
    axes[1].set_title("Fig. 0b — Magnetometer After Hard+Soft Iron Calibration")
    axes[1].set_xlabel("Mag X (calibrated)")
    axes[1].set_ylabel("Mag Y (calibrated)")
    axes[1].axis("equal")
    axes[1].grid(True)

    save_figure(fig, "fig0_mag_scatter.png")
    plt.close(fig)


def plot_fig1_mag_yaw(t, yaw_raw, yaw_cal):
    fig = plt.figure(figsize=(10, 4))
    plt.plot(t, np.rad2deg(yaw_raw), label="Raw magnetometer yaw", alpha=0.7)
    plt.plot(t, np.rad2deg(yaw_cal), label="Calibrated magnetometer yaw", linewidth=1.5)
    plt.title("Fig. 1 — Magnetometer Yaw Before/After Calibration")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (deg)")
    plt.grid(True)
    plt.legend()
    save_figure(fig, "fig1_mag_yaw.png")
    plt.close(fig)


def plot_fig2_gyro_yaw(t, gyro_yaw):
    fig = plt.figure(figsize=(10, 4))
    plt.plot(t, np.rad2deg(gyro_yaw), label="Integrated gyro yaw")
    plt.title("Fig. 2 — Gyro Yaw (integrated)")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (deg)")
    plt.grid(True)
    plt.legend()
    save_figure(fig, "fig2_gyro_yaw.png")
    plt.close(fig)


def plot_fig3_complementary(t, lpf_mag, hpf_gyro, fused, imu_heading):
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    labels = [
        ("Fig. 3a — LPF Magnetometer Yaw",    lpf_mag),
        ("Fig. 3b — HPF Gyro Yaw",             hpf_gyro),
        ("Fig. 3c — Complementary Filter Yaw", fused),
        ("Fig. 3d — IMU On-Board Heading",     imu_heading),
    ]
    for ax, (title, data) in zip(axes, labels):
        ax.plot(t, np.rad2deg(data))
        ax.set_title(title)
        ax.set_ylabel("Yaw (deg)")
        ax.grid(True)
    axes[-1].set_xlabel("Time (s)")
    save_figure(fig, "fig3_complementary_filter.png")
    plt.close(fig)


def plot_fig4_velocity_accel(t, raw_v, adj_v):
    fig = plt.figure(figsize=(10, 4))
    plt.plot(t, raw_v, label="Raw integrated velocity", alpha=0.6)
    plt.plot(t, adj_v, label="Bias-corrected velocity", linewidth=1.5)
    plt.title("Fig. 4 — Forward Velocity from Accelerometer")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.grid(True)
    plt.legend()
    save_figure(fig, "fig4_velocity_accel.png")
    plt.close(fig)


def plot_fig5_velocity_gps(t, gps_v):
    fig = plt.figure(figsize=(10, 4))
    plt.plot(t, gps_v, label="GPS velocity")
    plt.title("Fig. 5 — Forward Velocity from GPS")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.grid(True)
    plt.legend()
    save_figure(fig, "fig5_velocity_gps.png")
    plt.close(fig)


def plot_fig6_trajectory(gps_e, gps_n, imu_e, imu_n):
    """Two subplots (GPS | IMU) — both origin-aligned, IMU heading-matched to GPS."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    ge = gps_e - gps_e[np.isfinite(gps_e)][0]
    gn = gps_n - gps_n[np.isfinite(gps_n)][0]

    axes[0].plot(ge, gn, label="GPS track", color="steelblue")
    axes[0].set_title("Fig. 6a — GPS Trajectory")
    axes[0].set_xlabel("East (m)")
    axes[0].set_ylabel("North (m)")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(imu_e, imu_n, label="IMU dead reckoning", color="tomato")
    axes[1].set_title("Fig. 6b — IMU Dead-Reckoning Trajectory\n(heading aligned to GPS)")
    axes[1].set_xlabel("East (m)")
    axes[1].set_ylabel("North (m)")
    axes[1].axis("equal")
    axes[1].grid(True)
    axes[1].legend()

    save_figure(fig, "fig6_trajectory.png")
    plt.close(fig)


def plot_fig_q5_centripetal(t, omega_vx, ay_obs):
    """Q5: compare ω·Ẋ with ÿ_obs."""
    fig = plt.figure(figsize=(10, 4))
    plt.plot(t, omega_vx, label=r"$\omega \cdot \dot{X}$ (predicted lateral accel)", alpha=0.8)
    plt.plot(t, ay_obs,   label=r"$\ddot{y}_{obs}$ (accelerometer Y)", alpha=0.8)
    plt.title(r"Q5 — Centripetal Acceleration: $\omega \dot{X}$ vs $\ddot{y}_{obs}$")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.grid(True)
    plt.legend()
    save_figure(fig, "fig_q5_centripetal.png")
    plt.close(fig)


def plot_fig7_lidar_snapshots(lidar_samples: List[PointCloudSample]):
    for idx, frame_idx in enumerate(LIDAR_SNAPSHOT_INDICES):
        if frame_idx >= len(lidar_samples):
            continue
        xyz = filter_downsample_xyz(lidar_samples[frame_idx].xyz)
        fig = plt.figure(figsize=(9, 6))
        ax = fig.add_subplot(111, projection="3d")
        show = xyz[::max(1, len(xyz) // 5000)]
        sc = ax.scatter(show[:, 0], show[:, 1], show[:, 2], s=0.5,
                        c=show[:, 2], cmap="viridis")
        plt.colorbar(sc, ax=ax, label="Height (m)", pad=0.1)
        ax.set_title(f"Fig. 7 — LiDAR Point Cloud (frame {frame_idx})\n"
                     f"t = {lidar_samples[frame_idx].t:.2f} s | "
                     f"{len(xyz)} points after filtering")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        # Annotate approximate features
        ax.text2D(0.02, 0.95,
                  "Ground plane: low Z cluster\nBuildings/vehicles: high Z returns",
                  transform=ax.transAxes, fontsize=8,
                  bbox=dict(boxstyle="round", fc="white", alpha=0.7))
        save_figure(fig, f"fig7_lidar_snapshot_{idx + 1}.png")
        plt.close(fig)


def plot_fig8_compare_all(gps_e, gps_n, imu_e, imu_n, lidar_x, lidar_y):
    fig = plt.figure(figsize=(9, 7))
    plt.plot(gps_e   - gps_e[0],   gps_n   - gps_n[0],   label="GPS",            lw=1.5)
    plt.plot(imu_e   - imu_e[0],   imu_n   - imu_n[0],   label="IMU dead reckoning", lw=1.5)
    plt.plot(lidar_x - lidar_x[0], lidar_y - lidar_y[0], label="LiDAR ICP odometry", lw=1.5)
    plt.title("Fig. 8 — GPS vs IMU vs LiDAR Odometry (all origin-aligned)")
    plt.xlabel("East/X (m)")
    plt.ylabel("North/Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    save_figure(fig, "fig8_compare_all.png")
    plt.close(fig)


def plot_fig9_camera_snapshots(camera_samples: List[CameraSample],
                               event_labels: Optional[List[str]] = None):
    if len(camera_samples) == 0:
        return

    total = len(camera_samples)
    indices = np.linspace(0, total - 1, NUM_CAMERA_FRAMES, dtype=int)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    axes = axes.flatten()
    t0 = camera_samples[0].t

    default_events = [
        "Route start — stationary baseline",
        "First major turn / acceleration",
        "Mid-route — possible GPS multipath (tall buildings)",
        "Return leg — approaching start point",
    ]

    for i, (ax, frame_idx) in enumerate(zip(axes, indices)):
        sample = camera_samples[frame_idx]
        rgb = cv2.cvtColor(sample.image, cv2.COLOR_BGR2RGB)
        ax.imshow(rgb)
        label = (event_labels[i] if event_labels and i < len(event_labels)
                 else default_events[i])
        ax.set_title(f"t = {sample.t - t0:.1f} s\n{label}", fontsize=9)
        ax.axis("off")

    fig.suptitle("Fig. 9 — Representative Camera Frames with Trajectory Annotations",
                 fontsize=11)
    save_figure(fig, "fig9_camera_snapshots.png")
    plt.close(fig)


def plot_fig10_radar_placeholder():
    fig = plt.figure(figsize=(8, 5))
    plt.text(0.5, 0.5,
             "Radar (Continental ARS548) was not physically installed\n"
             "on the NUance vehicle during data collection.\n"
             "Fig. 10 is not applicable.",
             ha="center", va="center", fontsize=12,
             bbox=dict(boxstyle="round", fc="#f0f0f0"))
    plt.title("Fig. 10 — Radar Detection Plot (N/A)")
    plt.axis("off")
    save_figure(fig, "fig10_radar_na.png")
    plt.close(fig)


# =============================================================================
# MAIN
# =============================================================================

def main():
    ensure_dir(OUTPUT_DIR)

    # ------------------------------------------------------------------
    # 1. Magnetometer calibration — external circles bag (VN-100 + GPS puck)
    # ------------------------------------------------------------------
    print("[1/8] Loading calibration IMU data from external circles bag...")
    imu_calib = series_from_imu(load_imu_series(BAG_CALIB_EXT, TOPICS["imu"]))
    if len(imu_calib["t"]) == 0:
        raise RuntimeError(f"No IMU samples in calibration bag: {BAG_CALIB_EXT}")
    if np.all(np.isnan(imu_calib["mx"])) or np.all(np.isnan(imu_calib["my"])):
        raise RuntimeError("No magnetometer data in calibration bag.")

    print("[2/8] Fitting hard+soft iron calibration (least-squares ellipse)...")
    center, semi_axes, eigenvectors = fit_mag_calibration(
        imu_calib["mx"][np.isfinite(imu_calib["mx"])],
        imu_calib["my"][np.isfinite(imu_calib["my"])],
    )
    mx_calib, my_calib = apply_mag_calibration(
        imu_calib["mx"], imu_calib["my"], center, semi_axes, eigenvectors
    )
    plot_fig0_mag_scatter(imu_calib["mx"], imu_calib["my"], mx_calib, my_calib)
    print(f"  Hard-iron offset : ({center[0]:.4f}, {center[1]:.4f})")
    print(f"  Soft-iron axes   : ({semi_axes[0]:.4f}, {semi_axes[1]:.4f})")

    # ------------------------------------------------------------------
    # 3. Load IMU/GPS driving data — external city bag (Figs 1–6)
    # ------------------------------------------------------------------
    print("[3/8] Loading driving IMU + GPS data from external city bag...")
    imu_drive = series_from_imu(load_imu_series(BAG_DRIVE_EXT, TOPICS["imu"]))
    gps_drive = series_from_gps(load_gps_series(BAG_DRIVE_EXT, TOPICS["gps"]))

    if len(imu_drive["t"]) == 0:
        raise RuntimeError(f"No IMU samples in driving bag: {BAG_DRIVE_EXT}")
    if len(gps_drive["t"]) == 0:
        raise RuntimeError(f"No GPS samples in driving bag: {BAG_DRIVE_EXT}")
    if np.all(np.isnan(imu_drive["mx"])) or np.all(np.isnan(imu_drive["my"])):
        raise RuntimeError("No magnetometer data in driving bag.")
    if np.all(np.isnan(gps_drive["e"])) or np.all(np.isnan(gps_drive["n"])):
        raise RuntimeError("GPS produced no East/North values. Check extract_gps_sample().")

    print(f"  IMU samples: {len(imu_drive['t'])} | GPS samples: {len(gps_drive['t'])}")

    # Apply calibration to driving magnetometer
    mx_drive_cal, my_drive_cal = apply_mag_calibration(
        imu_drive["mx"], imu_drive["my"], center, semi_axes, eigenvectors
    )
    yaw_mag_raw = yaw_from_mag(imu_drive["mx"], imu_drive["my"])
    yaw_mag_cal = yaw_from_mag(mx_drive_cal, my_drive_cal)
    plot_fig1_mag_yaw(imu_drive["t"], yaw_mag_raw, yaw_mag_cal)

    # ------------------------------------------------------------------
    # 4. Yaw estimation + complementary filter
    # ------------------------------------------------------------------
    print("[4/8] Yaw estimation and complementary filter...")
    gyro_yaw = integrate_gyro_yaw(imu_drive["t"], imu_drive["gz"])
    plot_fig2_gyro_yaw(imu_drive["t"], gyro_yaw)

    fs_imu = estimate_sample_rate(imu_drive["t"])
    lpf_mag  = butter_filter(unwrap_angle(yaw_mag_cal), LPF_CUTOFF_HZ, fs_imu, "low",  FILTER_ORDER)
    hpf_gyro = butter_filter(unwrap_angle(gyro_yaw),    HPF_CUTOFF_HZ, fs_imu, "high", FILTER_ORDER)
    fused_yaw = wrap_to_pi(COMPLEMENTARY_ALPHA * lpf_mag + (1.0 - COMPLEMENTARY_ALPHA) * hpf_gyro)
    print(f"  IMU rate: {fs_imu:.1f} Hz | α={COMPLEMENTARY_ALPHA} | "
          f"LPF={LPF_CUTOFF_HZ} Hz | HPF={HPF_CUTOFF_HZ} Hz")

    imu_heading = imu_drive["heading_imu"].copy()
    if np.all(np.isnan(imu_heading)):
        print("  [NOTE] No quaternion heading — using calibrated mag yaw for Fig 3d.")
        imu_heading = yaw_mag_cal.copy()

    plot_fig3_complementary(imu_drive["t"], lpf_mag, hpf_gyro, fused_yaw, imu_heading)

    # ------------------------------------------------------------------
    # 5. Velocity estimation
    # ------------------------------------------------------------------
    print("[5/8] Velocity estimation...")
    raw_v, adj_v = estimate_forward_velocity_from_accel(imu_drive["t"], imu_drive["ax"])
    plot_fig4_velocity_accel(imu_drive["t"], raw_v, adj_v)

    gps_v = estimate_gps_velocity(gps_drive["t"], gps_drive["e"],
                                   gps_drive["n"], gps_drive["speed"])
    plot_fig5_velocity_gps(gps_drive["t"], gps_v)

    # Q5: centripetal acceleration comparison
    omega_vx = compute_centripetal(imu_drive["t"], imu_drive["gz"], adj_v)
    plot_fig_q5_centripetal(imu_drive["t"], omega_vx, imu_drive["ay"])
    diff_rms = np.sqrt(np.mean((omega_vx - imu_drive["ay"])**2))
    print(f"[Q5] RMS(ω·Ẋ − ÿ_obs) = {diff_rms:.4f} m/s²")

    # ------------------------------------------------------------------
    # 6. IMU trajectory vs GPS (Fig. 6) + Q7
    # ------------------------------------------------------------------
    print("[6/8] Building IMU trajectory...")
    imu_e_raw, imu_n_raw = build_imu_trajectory(
        imu_drive["t"], adj_v, wrap_to_pi(fused_yaw - fused_yaw[0])
    )

    valid_gps = np.isfinite(gps_drive["e"]) & np.isfinite(gps_drive["n"])
    gps_e_valid = gps_drive["e"][valid_gps] - gps_drive["e"][valid_gps][0]
    gps_n_valid = gps_drive["n"][valid_gps] - gps_drive["n"][valid_gps][0]

    if len(gps_e_valid) > 10 and len(imu_e_raw) > 10:
        imu_e_aligned, imu_n_aligned = align_imu_to_gps_heading(
            imu_e_raw, imu_n_raw, gps_e_valid, gps_n_valid
        )
    else:
        imu_e_aligned, imu_n_aligned = imu_e_raw, imu_n_raw

    plot_fig6_trajectory(gps_drive["e"], gps_drive["n"], imu_e_aligned, imu_n_aligned)

    # Q7
    max_dur, end_t = compute_gps_imu_agreement(
        gps_drive["e"], gps_drive["n"], gps_drive["t"],
        imu_e_aligned, imu_n_aligned, imu_drive["t"],
    )
    print(f"[Q7] Longest GPS–IMU agreement <2 m: {max_dur:.1f} s "
          f"(ends at t={end_t:.1f} s)")

    # ------------------------------------------------------------------
    # 7. LiDAR (Figs 7, 8) — vehicle bag
    # ------------------------------------------------------------------
    print("[7/8] LiDAR processing from vehicle bag...")
    # Load every 10th frame to cap memory usage; still gives ~200 frames for a 20-min drive.
    lidar_samples = load_lidar_series(BAG_DRIVE_VEH, TOPICS["lidar"], stride=10)
    if len(lidar_samples) > 0:
        print_lidar_stats(lidar_samples)
        plot_fig7_lidar_snapshots(lidar_samples)
        lidar_subset = lidar_samples[::max(1, len(lidar_samples) // 100)]
        lidar_x, lidar_y = estimate_lidar_odometry(lidar_subset)
        plot_fig8_compare_all(gps_e_valid, gps_n_valid,
                              imu_e_aligned, imu_n_aligned,
                              lidar_x, lidar_y)
    else:
        print("[WARN] No LiDAR frames in vehicle bag — skipping Figs 7 and 8.")

    # ------------------------------------------------------------------
    # 8. Camera (Fig. 9) — vehicle bag; Radar placeholder (Fig. 10)
    # ------------------------------------------------------------------
    print("[8/8] Camera from vehicle bag...")
    # stride=500 at 30 fps yields ~72 frames across a 20-min drive — enough for 4 snapshots.
    camera_samples = load_camera_series(BAG_DRIVE_VEH, TOPICS["camera"], stride=500)
    if len(camera_samples) > 0:
        plot_fig9_camera_snapshots(camera_samples)
    else:
        print("[WARN] No camera frames in vehicle bag — skipping Fig. 9.")

    plot_fig10_radar_placeholder()

    print()
    print("=" * 60)
    print("Done. All figures saved under:")
    print(f"  {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
