#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from utm import from_latlon

def parse_gngga_txt(filepath):
    latitudes    = []
    longitudes   = []
    altitudes    = []
    fix_qualities = []
    hdops        = []
    times_utc    = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line.startswith('$GNGGA'):
                continue
            
            if '*' in line:
                line = line[:line.index('*')]
            
            parts = line.split(',')
            if len(parts) < 10:
                continue

            try:
                utc_str    = parts[1]   
                lat_raw    = parts[2]   
                lat_dir    = parts[3]   
                lon_raw    = parts[4]   
                lon_dir    = parts[5]   
                fix_q_str  = parts[6]   
                hdop_str   = parts[8]   
                alt_str    = parts[9]   

                if not lat_raw or not lon_raw or not fix_q_str:
                    continue

                lat_deg = float(lat_raw[:2])
                lat_min = float(lat_raw[2:])
                lat = lat_deg + lat_min / 60.0
                if lat_dir == 'S':
                    lat = -lat

                lon_deg = float(lon_raw[:3])
                lon_min = float(lon_raw[3:])
                lon = lon_deg + lon_min / 60.0
                if lon_dir == 'W':
                    lon = -lon

                fix_q = int(fix_q_str)
                hdop  = float(hdop_str) if hdop_str else 0.0
                alt   = float(alt_str)  if alt_str  else 0.0

                h = float(utc_str[0:2])
                m = float(utc_str[2:4])
                s = float(utc_str[4:])
                t = h * 3600 + m * 60 + s

                latitudes.append(lat)
                longitudes.append(lon)
                altitudes.append(alt)
                fix_qualities.append(fix_q)
                hdops.append(hdop)
                times_utc.append(t)

            except Exception as e:
                continue  

    data = {
        'lat':         np.array(latitudes),
        'lon':         np.array(longitudes),
        'alt':         np.array(altitudes),
        'fix_quality': np.array(fix_qualities),
        'hdop':        np.array(hdops),
        'time':        np.array(times_utc),
    }

    eastings, northings = [], []
    for lat, lon in zip(data['lat'], data['lon']):
        e, n, _, _ = from_latlon(lat, lon)
        eastings.append(e)
        northings.append(n)
    data['easting']  = np.array(eastings)
    data['northing'] = np.array(northings)

    if len(data['time']) > 0:
        data['time'] = data['time'] - data['time'][0]

    return data

def centroid_error(easting, northing):
    c_e = np.mean(easting)
    c_n = np.mean(northing)
    dist = np.sqrt((easting - c_e)**2 + (northing - c_n)**2)
    rmse = np.sqrt(np.mean(dist**2))
    return c_e, c_n, dist, rmse


def line_fit_error(easting, northing):
    coeffs = np.polyfit(easting, northing, 1)
    m, b = coeffs
    dist = np.abs(m * easting - northing + b) / np.sqrt(m**2 + 1)
    rmse = np.sqrt(np.mean(dist**2))
    return coeffs, dist, rmse



def plot_stationary_NE(open_data, occ_data, prefix="RTK"):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(f'{prefix} Stationary Northing vs Easting', fontsize=13)

    datasets = [(open_data, 'Open', 'steelblue'),
                (occ_data,  'Occluded', 'darkorange')]

    for ax, (data, label, color) in zip(axes, datasets):
        c_e, c_n, dist, rmse = centroid_error(data['easting'], data['northing'])

        e_c = data['easting']  - c_e
        n_c = data['northing'] - c_n

        ax.scatter(e_c, n_c, s=6, color=color, alpha=0.5, label='Measurements')
        ax.scatter(0, 0, marker='+', s=150, color='red',
                   linewidths=2, label='Centroid (0,0)', zorder=5)

        ax.set_xlabel('Easting offset (m)')
        ax.set_ylabel('Northing offset (m)')
        ax.set_title(f'{prefix} {label}\nRMSE = {rmse:.4f} m')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')

        txt = (f'Centroid Easting:  {c_e:.2f} m\n'
               f'Centroid Northing: {c_n:.2f} m')
        ax.text(0.03, 0.97, txt, transform=ax.transAxes,
                fontsize=7, va='top',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    plt.savefig(f'{prefix}_stationary_NE.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"[Saved] {prefix}_stationary_NE.png")


def plot_altitude_time(open_data, occ_data, prefix="RTK"):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4))
    fig.suptitle(f'{prefix} Stationary Altitude vs Time', fontsize=13)

    datasets = [(open_data, 'Open', 'steelblue'),
                (occ_data,  'Occluded', 'darkorange')]

    for ax, (data, label, color) in zip(axes, datasets):
        ax.plot(data['time'], data['alt'], color=color, linewidth=0.9)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (m)')
        ax.set_title(f'{prefix} {label}')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{prefix}_altitude.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"[Saved] {prefix}_altitude.png")


def plot_histograms(open_data, occ_data, prefix="RTK"):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4))
    fig.suptitle(f'{prefix} Distance from Centroid Histogram', fontsize=13)

    datasets = [(open_data, 'Open', 'steelblue'),
                (occ_data,  'Occluded', 'darkorange')]

    for ax, (data, label, color) in zip(axes, datasets):
        _, _, dist, rmse = centroid_error(data['easting'], data['northing'])

        ax.hist(dist, bins=30, color=color, edgecolor='white',
                alpha=0.8, label='Distance')
        ax.axvline(np.mean(dist), color='red', linestyle='--',
                   linewidth=1.5, label=f'Mean = {np.mean(dist):.4f} m')
        ax.axvline(rmse, color='black', linestyle=':',
                   linewidth=1.5, label=f'RMSE = {rmse:.4f} m')
        ax.set_xlabel('Euclidean distance from centroid (m)')
        ax.set_ylabel('Count')
        ax.set_title(f'{prefix} {label}')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{prefix}_histogram.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"[Saved] {prefix}_histogram.png")


def plot_walking_NE(walk_data, prefix="RTK"):
    fig, ax = plt.subplots(figsize=(8, 6))

    e = walk_data['easting']
    n = walk_data['northing']
    coeffs, dist, rmse = line_fit_error(e, n)
    m, b = coeffs

    sc = ax.scatter(e, n, c=walk_data['time'], cmap='plasma',
                    s=15, zorder=3, label='GPS track')
    plt.colorbar(sc, ax=ax, label='Time (s)')

    e_line = np.linspace(e.min(), e.max(), 200)
    n_line = m * e_line + b
    ax.plot(e_line, n_line, 'r-', linewidth=2,
            label=f'Best fit  RMSE = {rmse:.4f} m')

    ax.scatter(e[0],  n[0],  marker='o', s=80, c='lime',
               zorder=5, label='Start')
    ax.scatter(e[-1], n[-1], marker='s', s=80, c='red',
               zorder=5, label='End')

    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_title(f'{prefix} Walking - Northing vs Easting')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{prefix}_walking_NE.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"[Saved] {prefix}_walking_NE.png")
    return rmse


def plot_walking_altitude(walk_data, prefix="RTK"):
    fig, ax = plt.subplots(figsize=(8, 4))

    ax.plot(walk_data['time'], walk_data['alt'],
            color='seagreen', linewidth=1.2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title(f'{prefix} Walking - Altitude vs Time')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{prefix}_walking_altitude.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"[Saved] {prefix}_walking_altitude.png")



if __name__ == '__main__':


    print("=" * 50)
    print("Loading RTK datasets...")
    print("=" * 50)

    occ_rtk  = parse_gngga_txt('../data/occludedRTK.txt')
    open_rtk = parse_gngga_txt('../data/openRTK.txt')
    walk_rtk = parse_gngga_txt('../data/walkingRTK.txt')

    print(f"occludedRTK : {len(occ_rtk['lat']):4d} points")
    print(f"openRTK     : {len(open_rtk['lat']):4d} points")
    print(f"walkingRTK  : {len(walk_rtk['lat']):4d} points")

    print("\nGenerating RTK plots...")

    plot_stationary_NE  (open_rtk, occ_rtk,  prefix="RTK")
    plot_altitude_time  (open_rtk, occ_rtk,  prefix="RTK")
    plot_histograms     (open_rtk, occ_rtk,  prefix="RTK")
    plot_walking_NE     (walk_rtk,            prefix="RTK")
    plot_walking_altitude(walk_rtk,           prefix="RTK")


    print("\n" + "=" * 50)
    print("NUMERICAL RESULTS FOR REPORT")
    print("=" * 50)


    _, _, _, rmse_open_rtk = centroid_error(
        open_rtk['easting'], open_rtk['northing'])
    _, _, _, rmse_occ_rtk  = centroid_error(
        occ_rtk['easting'],  occ_rtk['northing'])

 
    _, _, rmse_walk_rtk = line_fit_error(
        walk_rtk['easting'], walk_rtk['northing'])

    print(f"\n--- RTK Stationary ---")
    print(f"Open     RMSE : {rmse_open_rtk:.4f} m")
    print(f"Occluded RMSE : {rmse_occ_rtk:.4f} m")
    print(f"\n--- RTK Walking ---")
    print(f"Walking  RMSE : {rmse_walk_rtk:.4f} m")

    print(f"\n--- HDOP Statistics ---")
    print(f"Open     mean HDOP : {np.mean(open_rtk['hdop']):.2f}  "
          f"std: {np.std(open_rtk['hdop']):.2f}")
    print(f"Occluded mean HDOP : {np.mean(occ_rtk['hdop']):.2f}  "
          f"std: {np.std(occ_rtk['hdop']):.2f}")

    print(f"\n--- Fix Quality Distribution ---")
    uq_o, ct_o = np.unique(open_rtk['fix_quality'],  return_counts=True)
    uq_c, ct_c = np.unique(occ_rtk['fix_quality'],   return_counts=True)
    print(f"Open     fix_quality: {dict(zip(uq_o, ct_o))}")
    print(f"Occluded fix_quality: {dict(zip(uq_c, ct_c))}")
    print()
    print("Fix quality codes:")
    print("  1 = Standalone GPS")
    print("  4 = RTK Fixed (Float)")
    print("  5 = RTK Fixed")


    print("\n" + "=" * 50)
    print("REMINDER: Also include Lab 2 plots in your report!")
    print("  - Standalone open stationary NE plot")
    print("  - Standalone occluded stationary NE plot")
    print("  - Standalone walking NE plot")
    print("  - All altitude plots")
    print("  - All histograms")
    print("=" * 50)