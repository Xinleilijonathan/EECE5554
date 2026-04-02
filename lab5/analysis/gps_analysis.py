#!/usr/bin/env python3

import os
from pathlib import Path
import argparse
import numpy as np
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores


# ============================================================
# Utilities
# ============================================================

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def try_get_attr_chain(obj, chains):
    """
    Try multiple possible attribute chains and return the first one that works.
    Example chain: ["header", "stamp", "sec"]
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


# ============================================================
# Flexible field readers for Customgps
# ============================================================

def get_lat(msg):
    return float(try_get_attr_chain(msg, [
        ["latitude"],
        ["lat"],
        ["gps", "latitude"],
        ["gps", "lat"],
    ]))


def get_lon(msg):
    return float(try_get_attr_chain(msg, [
        ["longitude"],
        ["lon"],
        ["gps", "longitude"],
        ["gps", "lon"],
    ]))


def get_alt(msg):
    return float(try_get_attr_chain(msg, [
        ["altitude"],
        ["alt"],
        ["gps", "altitude"],
        ["gps", "alt"],
    ]))


def get_hdop(msg):
    return float(try_get_attr_chain(msg, [
        ["hdop"],
        ["gps", "hdop"],
    ]))


def get_utm_easting(msg):
    return float(try_get_attr_chain(msg, [
        ["utm_easting"],
        ["easting"],
        ["utm_x"],
        ["x"],
        ["utm", "easting"],
    ]))


def get_utm_northing(msg):
    return float(try_get_attr_chain(msg, [
        ["utm_northing"],
        ["northing"],
        ["utm_y"],
        ["y"],
        ["utm", "northing"],
    ]))


def get_fix_quality(msg):
    """
    Optional field. Some Customgps messages have it, some do not.
    """
    try:
        return int(try_get_attr_chain(msg, [
            ["fix_quality"],
            ["quality"],
            ["gps", "fix_quality"],
        ]))
    except Exception:
        return None


def get_utc_if_present(msg):
    """
    Optional field. Some custom GPS messages store UTC as a float/string/int.
    """
    try:
        return try_get_attr_chain(msg, [
            ["utc"],
            ["gps", "utc"],
            ["timestamp"],
        ])
    except Exception:
        return None


# ============================================================
# Read bag
# ============================================================

def read_gps_bag(bag_path, gps_topic="/gps"):
    typestore = get_typestore(Stores.ROS2_JAZZY)

    timestamps = []
    lat_list = []
    lon_list = []
    alt_list = []
    hdop_list = []
    utm_e_list = []
    utm_n_list = []
    fix_quality_list = []
    utc_list = []

    bag_path = Path(bag_path)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == gps_topic]

        if not connections:
            raise RuntimeError(f"Topic {gps_topic} not found in bag {bag_path}")

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            lat = get_lat(msg)
            lon = get_lon(msg)
            alt = get_alt(msg)
            hdop = get_hdop(msg)
            utm_e = get_utm_easting(msg)
            utm_n = get_utm_northing(msg)
            fix_quality = get_fix_quality(msg)
            utc_value = get_utc_if_present(msg)

            timestamps.append(timestamp * 1e-9)   # ns -> s
            lat_list.append(lat)
            lon_list.append(lon)
            alt_list.append(alt)
            hdop_list.append(hdop)
            utm_e_list.append(utm_e)
            utm_n_list.append(utm_n)
            fix_quality_list.append(fix_quality)
            utc_list.append(utc_value)

    t = np.array(timestamps)
    t = t - t[0]

    return {
        "t": t,
        "lat": np.array(lat_list),
        "lon": np.array(lon_list),
        "alt": np.array(alt_list),
        "hdop": np.array(hdop_list),
        "utm_e": np.array(utm_e_list),
        "utm_n": np.array(utm_n_list),
        "fix_quality": fix_quality_list,
        "utc": utc_list,
    }


# ============================================================
# Metrics
# ============================================================

def compute_basic_stats(data):
    utm_e = data["utm_e"]
    utm_n = data["utm_n"]
    alt = data["alt"]
    hdop = data["hdop"]

    stats = {}

    stats["num_points"] = len(data["t"])
    stats["duration_sec"] = float(data["t"][-1]) if len(data["t"]) > 0 else 0.0

    stats["mean_hdop"] = float(np.mean(hdop))
    stats["max_hdop"] = float(np.max(hdop))
    stats["min_hdop"] = float(np.min(hdop))

    stats["mean_alt"] = float(np.mean(alt))
    stats["alt_range"] = float(np.max(alt) - np.min(alt))

    # Path length in UTM plane
    if len(utm_e) >= 2:
        de = np.diff(utm_e)
        dn = np.diff(utm_n)
        segment_lengths = np.sqrt(de * de + dn * dn)
        stats["path_length_m"] = float(np.sum(segment_lengths))
    else:
        stats["path_length_m"] = 0.0

    # Relative trajectory spread
    stats["e_range_m"] = float(np.max(utm_e) - np.min(utm_e))
    stats["n_range_m"] = float(np.max(utm_n) - np.min(utm_n))

    return stats


# ============================================================
# Plotting
# ============================================================

def save_time_plot(t, y, ylabel, title, outfile):
    plt.figure(figsize=(10, 4))
    plt.plot(t, y, marker='o', markersize=3)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_trajectory_plot(utm_e, utm_n, title, outfile, relative=True):
    if relative:
        e = utm_e - utm_e[0]
        n = utm_n - utm_n[0]
        xlabel = "Relative Easting (m)"
        ylabel = "Relative Northing (m)"
    else:
        e = utm_e
        n = utm_n
        xlabel = "UTM Easting (m)"
        ylabel = "UTM Northing (m)"

    plt.figure(figsize=(7, 7))
    plt.plot(e, n, marker='o', markersize=3)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outfile, dpi=200)
    plt.close()


def save_summary_text(stats, outfile):
    with open(outfile, "w", encoding="utf-8") as f:
        f.write("GPS Analysis Summary\n")
        f.write("====================\n\n")
        for key, value in stats.items():
            f.write(f"{key}: {value}\n")


# ============================================================
# Per-dataset analysis
# ============================================================

def analyze_gps_dataset(name, bag_path, gps_topic, output_dir):
    ensure_dir(output_dir)

    print(f"Reading GPS bag for {name} ...")
    data = read_gps_bag(bag_path, gps_topic=gps_topic)
    stats = compute_basic_stats(data)

    t = data["t"]

    save_time_plot(
        t, data["lat"],
        "Latitude (deg)",
        f"{name} - Latitude vs Time",
        os.path.join(output_dir, f"{name}_gps_lat_vs_time.png")
    )

    save_time_plot(
        t, data["lon"],
        "Longitude (deg)",
        f"{name} - Longitude vs Time",
        os.path.join(output_dir, f"{name}_gps_lon_vs_time.png")
    )

    save_time_plot(
        t, data["alt"],
        "Altitude (m)",
        f"{name} - Altitude vs Time",
        os.path.join(output_dir, f"{name}_gps_alt_vs_time.png")
    )

    save_time_plot(
        t, data["hdop"],
        "HDOP",
        f"{name} - HDOP vs Time",
        os.path.join(output_dir, f"{name}_gps_hdop_vs_time.png")
    )

    save_trajectory_plot(
        data["utm_e"], data["utm_n"],
        f"{name} - UTM Trajectory",
        os.path.join(output_dir, f"{name}_gps_utm_trajectory.png"),
        relative=False
    )

    save_trajectory_plot(
        data["utm_e"], data["utm_n"],
        f"{name} - Relative UTM Trajectory",
        os.path.join(output_dir, f"{name}_gps_relative_trajectory.png"),
        relative=True
    )

    save_summary_text(
        stats,
        os.path.join(output_dir, f"{name}_gps_summary.txt")
    )

    return data, stats


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--circle_bag", required=True, help="Path to circle bag folder")
    parser.add_argument("--square_bag", required=True, help="Path to square bag folder")
    parser.add_argument("--gps_topic", default="/gps", help="GPS topic name")
    parser.add_argument("--output_dir", default="./gps_outputs", help="Output directory")
    args = parser.parse_args()

    ensure_dir(args.output_dir)

    circle_out = os.path.join(args.output_dir, "circle")
    square_out = os.path.join(args.output_dir, "square")

    circle_data, circle_stats = analyze_gps_dataset(
        name="circle",
        bag_path=args.circle_bag,
        gps_topic=args.gps_topic,
        output_dir=circle_out
    )

    square_data, square_stats = analyze_gps_dataset(
        name="square",
        bag_path=args.square_bag,
        gps_topic=args.gps_topic,
        output_dir=square_out
    )

    print("\nDone.\n")

    print("Circle GPS stats:")
    for k, v in circle_stats.items():
        print(f"  {k}: {v}")

    print("\nSquare GPS stats:")
    for k, v in square_stats.items():
        print(f"  {k}: {v}")

    print(f"\nPlots saved under: {args.output_dir}")


if __name__ == "__main__":
    main()