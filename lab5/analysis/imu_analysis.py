#!/usr/bin/env python3

import os
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
from pathlib import Path


# ============================================================
# Helper functions
# ============================================================

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def unwrap_angle(angle_rad):
    return np.unwrap(angle_rad)


def integrate_trapz(y, t):
    """
    Cumulative trapezoidal integration without scipy.
    y: np.array
    t: np.array
    """
    out = np.zeros_like(y)
    for i in range(1, len(y)):
        dt = t[i] - t[i - 1]
        out[i] = out[i - 1] + 0.5 * (y[i] + y[i - 1]) * dt
    return out


def lowpass_simple(x, window_size=5):
    """
    Very simple moving average filter.
    """
    if window_size <= 1:
        return x.copy()
    kernel = np.ones(window_size) / window_size
    return np.convolve(x, kernel, mode='same')


# ============================================================
# Flexible field readers for custom VectorNav message
# ============================================================

def try_get_attr_chain(obj, chains):
    """
    Try multiple attribute chains until one works.
    Example chain: ["imu", "angular_velocity", "x"]
    """
    for chain in chains:
        cur = obj
        ok = True
        for attr in chain:
            if not hasattr(cur, attr):
                ok = False
                break
            cur = getattr(cur, attr)
        if ok:
            return cur
    raise AttributeError(f"Could not find any of these attribute chains: {chains}")


def get_mag_xyz(msg):
    """
    Try common magnetometer field layouts.
    """
    mx = try_get_attr_chain(msg, [
        ["mag_field", "magnetic_field", "x"],
        ["magnetic_field", "x"],
        ["mag", "x"],
        ["mag_field", "x"],
        ["field", "x"],
    ])
    my = try_get_attr_chain(msg, [
        ["mag_field", "magnetic_field", "y"],
        ["magnetic_field", "y"],
        ["mag", "y"],
        ["mag_field", "y"],
        ["field", "y"],
    ])
    mz = try_get_attr_chain(msg, [
        ["mag_field", "magnetic_field", "z"],
        ["magnetic_field", "z"],
        ["mag", "z"],
        ["mag_field", "z"],
        ["field", "z"],
    ])
    return float(mx), float(my), float(mz)


def get_gyro_xyz(msg):
    """
    Try common gyro field layouts.
    """
    gx = try_get_attr_chain(msg, [
        ["imu", "angular_velocity", "x"],
        ["angular_velocity", "x"],
        ["gyro", "x"],
        ["gyroscope", "x"],
        ["angular_rates", "x"],
    ])
    gy = try_get_attr_chain(msg, [
        ["imu", "angular_velocity", "y"],
        ["angular_velocity", "y"],
        ["gyro", "y"],
        ["gyroscope", "y"],
        ["angular_rates", "y"],
    ])
    gz = try_get_attr_chain(msg, [
        ["imu", "angular_velocity", "z"],
        ["angular_velocity", "z"],
        ["gyro", "z"],
        ["gyroscope", "z"],
        ["angular_rates", "z"],
    ])
    return float(gx), float(gy), float(gz)


def get_acc_xyz(msg):
    """
    Try common accelerometer field layouts.
    """
    ax = try_get_attr_chain(msg, [
        ["imu", "linear_acceleration", "x"],
        ["linear_acceleration", "x"],
        ["accel", "x"],
        ["accelerometer", "x"],
        ["acceleration", "x"],
    ])
    ay = try_get_attr_chain(msg, [
        ["imu", "linear_acceleration", "y"],
        ["linear_acceleration", "y"],
        ["accel", "y"],
        ["accelerometer", "y"],
        ["acceleration", "y"],
    ])
    az = try_get_attr_chain(msg, [
        ["imu", "linear_acceleration", "z"],
        ["linear_acceleration", "z"],
        ["accel", "z"],
        ["accelerometer", "z"],
        ["acceleration", "z"],
    ])
    return float(ax), float(ay), float(az)


# ============================================================
# Bag reading
# ============================================================

def read_imu_bag(bag_path, imu_topic="/imu"):
    """
    Read IMU bag and return numpy arrays:
      t, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
    """
    typestore = get_typestore(Stores.ROS2_JAZZY)

    timestamps = []
    mag_x = []
    mag_y = []
    mag_z = []
    gyro_x = []
    gyro_y = []
    gyro_z = []
    acc_x = []
    acc_y = []
    acc_z = []

    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == imu_topic]

        if not connections:
            raise RuntimeError(f"Topic {imu_topic} not found in bag {bag_path}")

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            try:
                mx, my, mz = get_mag_xyz(msg)
                gx, gy, gz = get_gyro_xyz(msg)
                ax, ay, az = get_acc_xyz(msg)
            except Exception as e:
                raise RuntimeError(
                    f"Failed to parse IMU fields from message type {connection.msgtype}: {e}"
                )

            timestamps.append(timestamp * 1e-9)  # ns -> sec
            mag_x.append(mx)
            mag_y.append(my)
            mag_z.append(mz)
            gyro_x.append(gx)
            gyro_y.append(gy)
            gyro_z.append(gz)
            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)

    t = np.array(timestamps)
    t = t - t[0]

    return {
        "t": t,
        "mag_x": np.array(mag_x),
        "mag_y": np.array(mag_y),
        "mag_z": np.array(mag_z),
        "gyro_x": np.array(gyro_x),
        "gyro_y": np.array(gyro_y),
        "gyro_z": np.array(gyro_z),
        "acc_x": np.array(acc_x),
        "acc_y": np.array(acc_y),
        "acc_z": np.array(acc_z),
    }


# ============================================================
# Magnetometer calibration
# ============================================================

def calibrate_mag_hard_iron(mx, my):
    """
    Very simple 2D hard-iron calibration using circle center offset.
    This is often enough for a lab unless you want full ellipse fitting.
    """
    offset_x = 0.5 * (np.max(mx) + np.min(mx))
    offset_y = 0.5 * (np.max(my) + np.min(my))

    mx_cal = mx - offset_x
    my_cal = my - offset_y

    return mx_cal, my_cal, offset_x, offset_y


# ============================================================
# Heading / motion reconstruction
# ============================================================

def compute_headings(data, mx_cal, my_cal):
    """
    Magnetometer heading and gyro integrated heading.
    Lab guide says use atan2(X/Y), but in practice most people use atan2(Y, X).
    To match your handout wording, we keep atan2(mx, my).
    If your plot looks rotated by 90 degrees, switch this.
    """
    t = data["t"]

    # Magnetometer heading
    heading_mag = np.arctan2(mx_cal, my_cal)
    heading_mag = unwrap_angle(heading_mag)

    # Gyro integrated heading around Z
    yaw_rate = data["gyro_z"]
    heading_gyro = integrate_trapz(yaw_rate, t)
    heading_gyro = unwrap_angle(heading_gyro)

    return heading_mag, heading_gyro


def compute_axis_kinematics(acc, t, remove_mean=True):
    """
    Compute velocity and displacement from acceleration.
    """
    acc_proc = acc.copy()

    if remove_mean:
        acc_proc = acc_proc - np.mean(acc_proc)

    vel = integrate_trapz(acc_proc, t)
    vel = vel - np.mean(vel)

    disp = integrate_trapz(vel, t)

    return acc_proc, vel, disp


def compute_ne_trajectory_from_heading_and_forward_acc(acc_x, heading, t):
    """
    Use X-axis acceleration as forward acceleration in body frame.
    Integrate to forward velocity and then project to N/E using heading.
    This matches the spirit of the lab handout. :contentReference[oaicite:1]{index=1}
    """
    acc_forward = acc_x - np.mean(acc_x)
    vel_forward = integrate_trapz(acc_forward, t)
    vel_forward = vel_forward - np.mean(vel_forward)

    v_n = vel_forward * np.cos(heading)
    v_e = vel_forward * np.sin(heading)

    n = integrate_trapz(v_n, t)
    e = integrate_trapz(v_e, t)

    return vel_forward, n, e


# ============================================================
# Plotting functions
# ============================================================

def save_mag_cal_plot(mx, my, mx_cal, my_cal, title, outfile):
    plt.figure(figsize=(10, 5))

    plt.subplot(1, 2, 1)
    plt.plot(mx, my, '.', markersize=2)
    plt.xlabel("Mag X")
    plt.ylabel("Mag Y")
    plt.title(f"{title} - Before calibration")
    plt.axis("equal")
    plt.grid(True)
    plt.xticks(rotation=30)   # 👈 ADD THIS

    plt.subplot(1, 2, 2)
    plt.plot(mx_cal, my_cal, '.', markersize=2)
    plt.xlabel("Mag X calibrated")
    plt.ylabel("Mag Y calibrated")
    plt.title(f"{title} - After calibration")
    plt.axis("equal")
    plt.grid(True)
    plt.xticks(rotation=30)   # 👈 ADD THIS

    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_rate_and_angle_plot(t, rate, angle, axis_name, title, outfile):
    plt.figure(figsize=(10, 6))

    plt.subplot(2, 1, 1)
    plt.plot(t, rate)
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (rad/s)")
    plt.title(f"{title} - Gyro rotational rate around {axis_name}")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(t, angle)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.title(f"{title} - Integrated rotation around {axis_name}")
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_z_heading_plot(t, gz, yaw_gyro, heading_mag, title, outfile):
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, gz)
    plt.xlabel("Time (s)")
    plt.ylabel("Rate Z (rad/s)")
    plt.title(f"{title} - Rotational rate around Z")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, yaw_gyro)
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw from gyro (rad)")
    plt.title(f"{title} - Rotation from gyro around Z")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, heading_mag)
    plt.xlabel("Time (s)")
    plt.ylabel("Heading from magnetometer (rad)")
    plt.title(f"{title} - Magnetometer heading")
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_acc_vel_plot(t, acc, vel, axis_name, title, outfile):
    plt.figure(figsize=(10, 6))

    plt.subplot(2, 1, 1)
    plt.plot(t, acc)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title(f"{title} - Acceleration {axis_name}")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(t, vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title(f"{title} - Velocity {axis_name}")
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_acc_vel_disp_plot(t, acc, vel, disp, axis_name, title, outfile):
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, acc)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title(f"{title} - Acceleration {axis_name}")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title(f"{title} - Velocity {axis_name}")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, disp)
    plt.xlabel("Time (s)")
    plt.ylabel("Displacement (m)")
    plt.title(f"{title} - Displacement {axis_name}")
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_ne_plot(e_mag, n_mag, e_gyro, n_gyro, title, outfile):
    plt.figure(figsize=(7, 7))
    plt.plot(e_mag, n_mag, '.', markersize=2, label="Using magnetometer heading")
    plt.plot(e_gyro, n_gyro, '.', markersize=2, label="Using gyro heading")
    plt.xlabel("E (m)")
    plt.ylabel("N (m)")
    plt.title(f"{title} - N vs E trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


# ============================================================
# Main dataset analysis
# ============================================================

def analyze_dataset(name, data, outdir, circle_cal=None, do_square_style=False):
    ensure_dir(outdir)

    t = data["t"]

    # --------------------------------------------------------
    # Magnetometer calibration
    # --------------------------------------------------------
    if circle_cal is None:
        mx_cal, my_cal, offset_x, offset_y = calibrate_mag_hard_iron(
            data["mag_x"], data["mag_y"]
        )
    else:
        offset_x, offset_y = circle_cal
        mx_cal = data["mag_x"] - offset_x
        my_cal = data["mag_y"] - offset_y

    save_mag_cal_plot(
        data["mag_x"],
        data["mag_y"],
        mx_cal,
        my_cal,
        name,
        os.path.join(outdir, f"{name}_fig_mag_calibration.png"),
    )

    # --------------------------------------------------------
    # Gyro integrated angles
    # --------------------------------------------------------
    roll_gyro = integrate_trapz(data["gyro_x"], t)
    pitch_gyro = integrate_trapz(data["gyro_y"], t)
    yaw_gyro = integrate_trapz(data["gyro_z"], t)

    save_rate_and_angle_plot(
        t,
        data["gyro_x"],
        roll_gyro,
        "X",
        name,
        os.path.join(outdir, f"{name}_fig_gyro_x.png"),
    )
    save_rate_and_angle_plot(
        t,
        data["gyro_y"],
        pitch_gyro,
        "Y",
        name,
        os.path.join(outdir, f"{name}_fig_gyro_y.png"),
    )

    # --------------------------------------------------------
    # Heading
    # --------------------------------------------------------
    heading_mag, heading_gyro = compute_headings(data, mx_cal, my_cal)

    save_z_heading_plot(
        t,
        data["gyro_z"],
        yaw_gyro,
        heading_mag,
        name,
        os.path.join(outdir, f"{name}_fig_gyro_z_heading.png"),
    )

    # --------------------------------------------------------
    # Acc / vel / disp
    # --------------------------------------------------------
    acc_x_proc, vel_x, disp_x = compute_axis_kinematics(data["acc_x"], t)
    acc_y_proc, vel_y, disp_y = compute_axis_kinematics(data["acc_y"], t)
    acc_z_proc, vel_z, disp_z = compute_axis_kinematics(data["acc_z"], t)

    if do_square_style:
        save_acc_vel_disp_plot(
            t, acc_x_proc, vel_x, disp_x, "X", name,
            os.path.join(outdir, f"{name}_fig_acc_vel_disp_x.png")
        )
        save_acc_vel_disp_plot(
            t, acc_y_proc, vel_y, disp_y, "Y", name,
            os.path.join(outdir, f"{name}_fig_acc_vel_disp_y.png")
        )
        save_acc_vel_disp_plot(
            t, acc_z_proc, vel_z, disp_z, "Z", name,
            os.path.join(outdir, f"{name}_fig_acc_vel_disp_z.png")
        )
    else:
        save_acc_vel_plot(
            t, acc_x_proc, vel_x, "X", name,
            os.path.join(outdir, f"{name}_fig_acc_vel_x.png")
        )
        save_acc_vel_plot(
            t, acc_y_proc, vel_y, "Y", name,
            os.path.join(outdir, f"{name}_fig_acc_vel_y.png")
        )
        save_acc_vel_plot(
            t, acc_z_proc, vel_z, "Z", name,
            os.path.join(outdir, f"{name}_fig_acc_vel_z.png")
        )

    # --------------------------------------------------------
    # N/E trajectory using forward acceleration and heading
    # --------------------------------------------------------
    _, n_mag, e_mag = compute_ne_trajectory_from_heading_and_forward_acc(
        data["acc_x"], heading_mag, t
    )
    _, n_gyro, e_gyro = compute_ne_trajectory_from_heading_and_forward_acc(
        data["acc_x"], heading_gyro, t
    )

    save_ne_plot(
        e_mag,
        n_mag,
        e_gyro,
        n_gyro,
        name,
        os.path.join(outdir, f"{name}_fig_ne_trajectory.png"),
    )

    return {
        "mag_offset_x": offset_x,
        "mag_offset_y": offset_y,
        "heading_mag": heading_mag,
        "heading_gyro": heading_gyro,
    }


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--circle_bag", required=True, help="Path to circle bag folder")
    parser.add_argument("--square_bag", required=True, help="Path to square bag folder")
    parser.add_argument("--imu_topic", default="/imu", help="IMU topic name")
    parser.add_argument("--output_dir", default="outputs", help="Output directory")
    args = parser.parse_args()

    ensure_dir(args.output_dir)

    print("Reading circle bag...")
    circle_data = read_imu_bag(args.circle_bag, imu_topic=args.imu_topic)

    print("Reading square bag...")
    square_data = read_imu_bag(args.square_bag, imu_topic=args.imu_topic)

    # Analyze circle first and compute calibration from circle
    print("Analyzing circle bag...")
    circle_results = analyze_dataset(
        name="circle",
        data=circle_data,
        outdir=os.path.join(args.output_dir, "circle"),
        circle_cal=None,
        do_square_style=False,
    )

    circle_cal = (
        circle_results["mag_offset_x"],
        circle_results["mag_offset_y"],
    )

    # Apply circle calibration to square
    print("Analyzing square bag...")
    analyze_dataset(
        name="square",
        data=square_data,
        outdir=os.path.join(args.output_dir, "square"),
        circle_cal=circle_cal,
        do_square_style=True,
    )

    print("\nDone.")
    print(f"Circle calibration offsets:")
    print(f"  offset_x = {circle_cal[0]:.6f}")
    print(f"  offset_y = {circle_cal[1]:.6f}")
    print(f"Plots saved to: {args.output_dir}")


if __name__ == "__main__":
    main()