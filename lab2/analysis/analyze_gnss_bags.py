#!/usr/bin/env python3
"""
Analyze 3 ROS2 GNSS rosbag2 datasets (MCAP or SQLite3) and generate plots for Lab2 quiz.

One-file-per-question outputs:
- Q2: stationary northing vs easting (open + occluded), relative (subtract 1st sample)
- Q3: stationary altitude vs time (open + occluded) in ONE PNG (2 subplots)
- Q4: stationary error histogram from "known" position (open + occluded) in ONE PNG (2 subplots)
- Q7: walking northing vs easting (line200)
- Q8: walking altitude vs time (line200)

Also generates general plots:
- trajectory_utm_overlay.png, hdop_vs_time.png, altitude_vs_time.png, latitude_vs_time.png, longitude_vs_time.png

And straight-line metrics:
- cross_track_error_<line>.png, noise_hist_<line>.png, noise_drift_metrics_<line>.csv
- q5_stationary_error_metrics.csv

Usage:
  source /opt/ros/jazzy/setup.bash
  source ../gnss_env/bin/activate

  python analyze_gnss_bags.py \
    --bags ../data/rosbags/open_area_5min ../data/rosbags/occluded_area_5min ../data/rosbags/200m_line \
    --labels open occluded line200 \
    --out ./out_gnss \
    --topic /gps \
    --open_label open \
    --occluded_label occluded \
    --walk_label line200 \
    --line_label line200
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ROS2 Jazzy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def to_bag_dir(p: str) -> Path:
    path = Path(p).expanduser().resolve()
    if path.is_dir():
        return path
    if path.is_file():
        return path.parent
    raise FileNotFoundError(f"Bag path not found: {p}")


def _pick_storage_id(bag_dir: Path) -> str:
    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    if any(bag_dir.glob("*.db3")):
        return "sqlite3"
    return "mcap"


def read_gps_from_bag(bag_dir: Path, topic: str) -> pd.DataFrame:
    rows = []

    storage_id = _pick_storage_id(bag_dir)
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    if topic not in type_map:
        available = sorted(type_map.keys())
        raise RuntimeError(f"Topic '{topic}' not in bag {bag_dir}. Available topics:\n{available}")

    msg_type_str = type_map[topic]
    MsgType = get_message(msg_type_str)

    t0 = None
    while reader.has_next():
        (t_name, data, _t_ns) = reader.read_next()
        if t_name != topic:
            continue

        msg = deserialize_message(data, MsgType)

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if t0 is None:
            t0 = stamp

        rows.append(
            {
                "stamp": stamp,
                "t": stamp - t0,
                "frame_id": str(msg.header.frame_id),
                "latitude": float(msg.latitude),
                "longitude": float(msg.longitude),
                "altitude": float(msg.altitude),
                "utm_easting": float(msg.utm_easting),
                "utm_northing": float(msg.utm_northing),
                "hdop": float(msg.hdop),
                "gpgga_read": str(getattr(msg, "gpgga_read", "")),
            }
        )

    df = pd.DataFrame(rows)
    if df.empty:
        raise RuntimeError(f"No messages read on {topic} from bag: {bag_dir}")

    return df.sort_values("stamp").reset_index(drop=True)


def fit_line_pca(x: np.ndarray, y: np.ndarray):
    pts = np.vstack([x, y]).T
    c = pts.mean(axis=0)
    pts0 = pts - c
    cov = np.cov(pts0.T)
    eigvals, eigvecs = np.linalg.eig(cov)
    v = eigvecs[:, np.argmax(eigvals)]
    v = v / np.linalg.norm(v)
    return c[0], c[1], v[0], v[1]


def cross_track_error(x, y, cx, cy, dx, dy):
    rx = x - cx
    ry = y - cy
    return dx * ry - dy * rx


def moving_average(a: np.ndarray, win: int):
    if win <= 1:
        return a
    win = min(win, len(a))
    kernel = np.ones(win) / win
    return np.convolve(a, kernel, mode="same")


def make_general_plots(dfs, labels, outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)

    # Trajectory overlay
    plt.figure()
    for df, lab in zip(dfs, labels):
        plt.plot(df["utm_easting"], df["utm_northing"], label=lab)
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.title("GNSS Trajectory (UTM) - Overlay")
    plt.legend()
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(outdir / "trajectory_utm_overlay.png", dpi=200)
    plt.close()

    # HDOP vs time
    plt.figure()
    for df, lab in zip(dfs, labels):
        plt.plot(df["t"], df["hdop"], label=lab)
    plt.xlabel("Time since start (s)")
    plt.ylabel("HDOP")
    plt.title("HDOP vs Time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "hdop_vs_time.png", dpi=200)
    plt.close()

    # Altitude vs time
    plt.figure()
    for df, lab in zip(dfs, labels):
        plt.plot(df["t"], df["altitude"], label=lab)
    plt.xlabel("Time since start (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude vs Time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "altitude_vs_time.png", dpi=200)
    plt.close()

    # Lat vs time
    plt.figure()
    for df, lab in zip(dfs, labels):
        plt.plot(df["t"], df["latitude"], label=lab)
    plt.xlabel("Time since start (s)")
    plt.ylabel("Latitude (deg)")
    plt.title("Latitude vs Time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "latitude_vs_time.png", dpi=200)
    plt.close()

    # Lon vs time
    plt.figure()
    for df, lab in zip(dfs, labels):
        plt.plot(df["t"], df["longitude"], label=lab)
    plt.xlabel("Time since start (s)")
    plt.ylabel("Longitude (deg)")
    plt.title("Longitude vs Time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "longitude_vs_time.png", dpi=200)
    plt.close()


def analyze_noise_drift(df_line: pd.DataFrame, outdir: Path, label: str, smooth_win: int = 51):
    outdir.mkdir(parents=True, exist_ok=True)

    x = df_line["utm_easting"].to_numpy()
    y = df_line["utm_northing"].to_numpy()
    t = df_line["t"].to_numpy()

    cx, cy, dx, dy = fit_line_pca(x, y)
    e = cross_track_error(x, y, cx, cy, dx, dy)

    e_smooth = moving_average(e, smooth_win)
    e_detrended = e - e_smooth

    noise_std = float(np.std(e_detrended))

    A = np.vstack([t, np.ones_like(t)]).T
    slope, intercept = np.linalg.lstsq(A, e_smooth, rcond=None)[0]
    drift_slope = float(slope)

    plt.figure()
    plt.plot(t, e, label="cross-track error (raw)")
    plt.plot(t, e_smooth, label=f"smoothed (win={smooth_win})")
    plt.xlabel("Time since start (s)")
    plt.ylabel("Cross-track error (m)")
    plt.title(f"Noise/Drift Analysis (Straight Line) - {label}")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / f"cross_track_error_{label}.png", dpi=200)
    plt.close()

    plt.figure()
    plt.hist(e_detrended, bins=50)
    plt.xlabel("Detrended cross-track error (m)")
    plt.ylabel("Count")
    plt.title(f"Noise Histogram - {label} (std={noise_std:.3f} m)")
    plt.tight_layout()
    plt.savefig(outdir / f"noise_hist_{label}.png", dpi=200)
    plt.close()

    metrics = {
        "label": label,
        "noise_std_m": noise_std,
        "drift_slope_m_per_s": drift_slope,
        "drift_slope_m_per_min": drift_slope * 60.0,
        "cross_track_mean_m": float(np.mean(e)),
        "cross_track_abs_mean_m": float(np.mean(np.abs(e))),
        "n_samples": int(len(df_line)),
        "smooth_window": int(smooth_win),
    }
    pd.DataFrame([metrics]).to_csv(outdir / f"noise_drift_metrics_{label}.csv", index=False)
    return metrics


def make_quiz_plots_one_file_per_question(
    df_open: pd.DataFrame,
    df_occ: pd.DataFrame,
    df_walk: pd.DataFrame,
    outdir: Path,
    known_e: float | None,
    known_n: float | None,
):
    outdir.mkdir(parents=True, exist_ok=True)

    # ---------------- Q2: stationary XY relative (subtract first sample) ----------------
    e0_open, n0_open = df_open["utm_easting"].iloc[0], df_open["utm_northing"].iloc[0]
    e0_occ, n0_occ = df_occ["utm_easting"].iloc[0], df_occ["utm_northing"].iloc[0]

    open_e_rel = df_open["utm_easting"] - e0_open
    open_n_rel = df_open["utm_northing"] - n0_open
    occ_e_rel = df_occ["utm_easting"] - e0_occ
    occ_n_rel = df_occ["utm_northing"] - n0_occ

    plt.figure()
    plt.scatter(open_e_rel, open_n_rel, s=10, label="open (relative)")
    plt.scatter(occ_e_rel, occ_n_rel, s=10, label="occluded (relative)")
    plt.xlabel("Relative Easting (m)")
    plt.ylabel("Relative Northing (m)")
    plt.title("Q2: Stationary Northing vs Easting (relative to first sample)")
    plt.legend()
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(outdir / "q2_stationary_xy_rel_open_occluded.png", dpi=200)
    plt.close()

    # ---------------- Q3: stationary Altitude vs Time in ONE PNG (2 subplots) ----------------
    fig, ax = plt.subplots(2, 1, figsize=(8, 8), sharex=False)
    ax[0].scatter(df_open["t"], df_open["altitude"], s=10)
    ax[0].set_title("Q3: Stationary Altitude vs Time (open)")
    ax[0].set_xlabel("Time since start (s)")
    ax[0].set_ylabel("Altitude (m)")

    ax[1].scatter(df_occ["t"], df_occ["altitude"], s=10)
    ax[1].set_title("Q3: Stationary Altitude vs Time (occluded)")
    ax[1].set_xlabel("Time since start (s)")
    ax[1].set_ylabel("Altitude (m)")

    fig.tight_layout()
    fig.savefig(outdir / "q3_stationary_alt_vs_time_open_occluded.png", dpi=200)
    plt.close(fig)

    # ---------------- Q4/Q5: stationary error from known position ----------------
    # If not provided, use open mean as reference.
    if known_e is None or known_n is None:
        E_ref = float(df_open["utm_easting"].mean())
        N_ref = float(df_open["utm_northing"].mean())
        print(f"[INFO] Using open mean as known reference: E_ref={E_ref:.3f}, N_ref={N_ref:.3f}")
    else:
        E_ref = float(known_e)
        N_ref = float(known_n)
        print(f"[INFO] Using provided known reference: E_ref={E_ref:.3f}, N_ref={N_ref:.3f}")

    open_err = np.sqrt((df_open["utm_easting"] - E_ref) ** 2 + (df_open["utm_northing"] - N_ref) ** 2)
    occ_err = np.sqrt((df_occ["utm_easting"] - E_ref) ** 2 + (df_occ["utm_northing"] - N_ref) ** 2)

    # ONE PNG with 2 subplots
    fig, ax = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    ax[0].hist(open_err, bins=40)
    ax[0].set_title("Q4: Stationary Error Histogram (open)")
    ax[0].set_ylabel("Count")

    ax[1].hist(occ_err, bins=40)
    ax[1].set_title("Q4: Stationary Error Histogram (occluded)")
    ax[1].set_xlabel("Horizontal error from reference (m)")
    ax[1].set_ylabel("Count")

    fig.tight_layout()
    fig.savefig(outdir / "q4_stationary_error_hist_open_occluded.png", dpi=200)
    plt.close(fig)

    # Save Q5 metrics CSV (one row per dataset)
    def _metrics(name: str, err: np.ndarray):
        return {
            "dataset": name,
            "reference_easting_m": E_ref,
            "reference_northing_m": N_ref,
            "mean_err_m": float(np.mean(err)),
            "median_err_m": float(np.median(err)),
            "rms_err_m": float(np.sqrt(np.mean(err**2))),
            "std_err_m": float(np.std(err)),
            "p95_err_m": float(np.percentile(err, 95)),
            "n_samples": int(len(err)),
        }

    q5 = pd.DataFrame([_metrics("open", open_err.to_numpy()), _metrics("occluded", occ_err.to_numpy())])
    q5.to_csv(outdir / "q5_stationary_error_metrics.csv", index=False)

    # ---------------- Q7: walking Northing vs Easting (line200) ----------------
    plt.figure()
    plt.scatter(df_walk["utm_easting"], df_walk["utm_northing"], s=10)
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.title("Q7: Walking Northing vs Easting")
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(outdir / "q7_walking_xy_line200.png", dpi=200)
    plt.close()

    # ---------------- Q8: walking Altitude vs Time (line200) ----------------
    plt.figure()
    plt.scatter(df_walk["t"], df_walk["altitude"], s=10)
    plt.xlabel("Time since start (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Q8: Walking Altitude vs Time")
    plt.tight_layout()
    plt.savefig(outdir / "q8_walking_alt_vs_time_line200.png", dpi=200)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", nargs="+", required=True, help="bag paths (directories or file paths)")
    parser.add_argument("--labels", nargs="+", required=True, help="labels for plotting (same count as bags)")
    parser.add_argument("--topic", default="/gps", help="topic name to analyze (default: /gps)")
    parser.add_argument("--out", default="out_gnss", help="output directory")

    # Map your datasets to quiz roles
    parser.add_argument("--open_label", default="open", help="label for stationary open-sky dataset")
    parser.add_argument("--occluded_label", default="occluded", help="label for stationary occluded dataset")
    parser.add_argument("--walk_label", default="line200", help="label for walking dataset")

    parser.add_argument("--line_label", default=None, help="label for straight-line noise/drift (often same as walk_label)")
    parser.add_argument("--smooth_win", type=int, default=51, help="moving average window for drift (odd preferred)")

    # Optional: if lab provides a known stationary location in UTM, put it here
    parser.add_argument("--known_easting", type=float, default=None, help="known/reference UTM easting (m)")
    parser.add_argument("--known_northing", type=float, default=None, help="known/reference UTM northing (m)")

    args = parser.parse_args()

    if len(args.bags) != len(args.labels):
        raise SystemExit("Error: --bags and --labels must have the same count.")
    if len(args.bags) < 1:
        raise SystemExit("Error: provide at least 1 bag.")

    bag_dirs = [to_bag_dir(p) for p in args.bags]
    outdir = Path(args.out).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    dfs = {}
    for p, lab in zip(bag_dirs, args.labels):
        print(f"[READ] {lab}: {p}")
        df = read_gps_from_bag(p, args.topic)
        dfs[lab] = df

        csv_path = outdir / f"{lab}_gps.csv"
        df.to_csv(csv_path, index=False)
        print(f"  -> saved {csv_path} (rows={len(df)})")

    # General overlay plots (nice for your report)
    make_general_plots([dfs[l] for l in args.labels], args.labels, outdir)

    # Quiz plots (one file per question)
    if args.open_label not in dfs or args.occluded_label not in dfs or args.walk_label not in dfs:
        raise SystemExit(
            f"Error: Need labels present for open/occluded/walk.\n"
            f"Got labels: {list(dfs.keys())}\n"
            f"Expected: open_label={args.open_label}, occluded_label={args.occluded_label}, walk_label={args.walk_label}"
        )

    make_quiz_plots_one_file_per_question(
        df_open=dfs[args.open_label],
        df_occ=dfs[args.occluded_label],
        df_walk=dfs[args.walk_label],
        outdir=outdir,
        known_e=args.known_easting,
        known_n=args.known_northing,
    )

    # Straight-line noise/drift analysis
    line_lab = args.line_label if args.line_label else args.walk_label
    if line_lab not in dfs:
        raise SystemExit(f"Error: line_label {line_lab} not found in labels {list(dfs.keys())}")

    metrics = analyze_noise_drift(dfs[line_lab], outdir, line_lab, smooth_win=args.smooth_win)

    print("\n=== Summary ===")
    print(f"Output folder: {outdir}")
    print("Generated PNGs:")
    for f in sorted(outdir.glob("*.png")):
        print(" -", f.name)

    print("\nNoise/Drift metrics (straight line):")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    print("\nSaved Q5 stationary error metrics:")
    print(" - q5_stationary_error_metrics.csv")


if __name__ == "__main__":
    main()
