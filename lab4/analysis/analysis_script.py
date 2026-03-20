#!/usr/bin/env python3
"""
EECE5554 Lab 4 - IMU Analysis Script
Requires: rosbags, numpy, matplotlib, allantools
Install:  pip install rosbags allantools numpy matplotlib --break-system-packages
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees
import allantools

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# ── CONFIG ────────────────────────────────────────────────────────────────────
STATIONARY_BAG = '/home/jonathancyriane/EECE5554/lab4/data/stationary_5min'
FREEFORM_BAG   = '/home/jonathancyriane/EECE5554/lab4/data/freeform_2min'
TOPIC          = '/imu'
# ─────────────────────────────────────────────────────────────────────────────

# Register the custom Vectornav message type
VECTORNAV_MSG = """
std_msgs/Header header
sensor_msgs/Imu imu
sensor_msgs/MagneticField mag_field
string raw_str
"""

typestore = get_typestore(Stores.ROS2_JAZZY)
typestore.register(get_types_from_msg(VECTORNAV_MSG, 'imu_msgs/msg/Vectornav'))


def quat_to_euler(w, x, y, z):
    """Convert quaternion to roll, pitch, yaw in degrees (ZYX convention)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = degrees(np.arctan2(sinr, cosr))

    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = degrees(np.arcsin(sinp))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = degrees(np.arctan2(siny, cosy))

    return roll, pitch, yaw


def read_bag(bag_path):
    """Read /imu topic from a ros2 bag."""
    data = {'t': [], 'gx': [], 'gy': [], 'gz': [],
            'ax': [], 'ay': [], 'az': [],
            'qw': [], 'qx': [], 'qy': [], 'qz': []}

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == TOPIC]
        if not connections:
            raise RuntimeError(f'Topic {TOPIC} not found in {bag_path}')

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            imu = msg.imu
            data['t'].append(timestamp * 1e-9)
            data['gx'].append(imu.angular_velocity.x)
            data['gy'].append(imu.angular_velocity.y)
            data['gz'].append(imu.angular_velocity.z)
            data['ax'].append(imu.linear_acceleration.x)
            data['ay'].append(imu.linear_acceleration.y)
            data['az'].append(imu.linear_acceleration.z)
            data['qw'].append(imu.orientation.w)
            data['qx'].append(imu.orientation.x)
            data['qy'].append(imu.orientation.y)
            data['qz'].append(imu.orientation.z)

    for k in data:
        data[k] = np.array(data[k])
    data['t'] -= data['t'][0]  # zero-base time

    # Gyro rad/s → deg/s
    data['gx_deg'] = np.degrees(data['gx'])
    data['gy_deg'] = np.degrees(data['gy'])
    data['gz_deg'] = np.degrees(data['gz'])

    # Quaternion → Euler
    rolls, pitches, yaws = [], [], []
    for w, x, y, z in zip(data['qw'], data['qx'], data['qy'], data['qz']):
        r, p, y_ = quat_to_euler(w, x, y, z)
        rolls.append(r); pitches.append(p); yaws.append(y_)
    data['roll']  = np.array(rolls)
    data['pitch'] = np.array(pitches)
    data['yaw']   = np.array(yaws)

    return data


# ── LOAD DATA ─────────────────────────────────────────────────────────────────
print('Reading stationary bag...')
stat = read_bag(STATIONARY_BAG)

print('Reading free-form bag...')
free = read_bag(FREEFORM_BAG)
print('Data loaded successfully!')


# ── FIG 0: Gyro rotational rate (deg/s) ──────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle('Fig 0 — Gyro Rotational Rate (stationary)')
for ax, key, label, color in zip(axes,
                                  ['gx_deg', 'gy_deg', 'gz_deg'],
                                  ['X', 'Y', 'Z'],
                                  ['tab:blue', 'tab:orange', 'tab:green']):
    ax.plot(stat['t'], stat[key], color=color, linewidth=0.5)
    ax.set_ylabel(f'ω_{label} (°/s)')
    ax.grid(True)
axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.savefig('fig0_gyro_rate.png', dpi=150)
plt.show()


# ── FIG 1: Accelerometer (m/s²) ──────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle('Fig 1 — Linear Acceleration (stationary)')
for ax, key, label, color in zip(axes,
                                  ['ax', 'ay', 'az'],
                                  ['X', 'Y', 'Z'],
                                  ['tab:blue', 'tab:orange', 'tab:green']):
    ax.plot(stat['t'], stat[key], color=color, linewidth=0.5)
    ax.set_ylabel(f'a_{label} (m/s²)')
    ax.grid(True)
axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.savefig('fig1_accel.png', dpi=150)
plt.show()


# ── FIG 2: VN orientation estimation (degrees) ───────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle('Fig 2 — VN Orientation Estimation (stationary)')
for ax, key, label, color in zip(axes,
                                  ['roll', 'pitch', 'yaw'],
                                  ['Roll', 'Pitch', 'Yaw'],
                                  ['tab:blue', 'tab:orange', 'tab:green']):
    ax.plot(stat['t'], stat[key], color=color, linewidth=0.5)
    ax.set_ylabel(f'{label} (°)')
    ax.grid(True)
axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.savefig('fig2_orientation.png', dpi=150)
plt.show()


# ── FIG 3: Allan Deviation (gyro x, y, z) ────────────────────────────────────
dt   = np.mean(np.diff(stat['t']))
rate = 1.0 / dt

fig, ax = plt.subplots(figsize=(10, 6))
fig.suptitle('Fig 3 — Gyro Allan Deviation')

for key, label, color in zip(['gx', 'gy', 'gz'],
                               ['X', 'Y', 'Z'],
                               ['tab:blue', 'tab:orange', 'tab:green']):
    taus, ad, _, _ = allantools.oadev(stat[key], rate=rate, data_type='freq')
    ax.loglog(taus, ad, label=f'Gyro {label}', color=color)

tau_ref = np.logspace(np.log10(taus[0]), np.log10(taus[-1]), 100)
ax.loglog(tau_ref, 0.001 / np.sqrt(tau_ref), 'k--', linewidth=0.8, label='Slope -½ (ARW)')
ax.loglog(tau_ref, np.full_like(tau_ref, 1e-4), 'k:', linewidth=0.8, label='Slope 0 (Bias stability)')

ax.set_xlabel('Averaging Time τ (s)')
ax.set_ylabel('Allan Deviation (rad/s)')
ax.legend()
ax.grid(True, which='both', linestyle='--', alpha=0.5)
plt.tight_layout()
plt.savefig('fig3_allan_deviation.png', dpi=150)
plt.show()


# ── FIG 4-6: Free-form clips (1-5 seconds each) ──────────────────────────────
total_time = free['t'][-1]
clip_starts   = [total_time * 0.15, total_time * 0.45, total_time * 0.75]
clip_duration = 3.0

for i, (start, fname) in enumerate(zip(clip_starts,
                                        ['fig4_clip1.png', 'fig5_clip2.png', 'fig6_clip3.png'])):
    mask   = (free['t'] >= start) & (free['t'] < start + clip_duration)
    t_clip = free['t'][mask] - free['t'][mask][0]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f'Fig {4+i} — Free-form clip {i+1} '
                 f'(t={start:.1f}–{start+clip_duration:.1f}s)')

    axes[0].plot(t_clip, free['gx_deg'][mask], label='ωx', color='tab:blue')
    axes[0].plot(t_clip, free['gy_deg'][mask], label='ωy', color='tab:orange')
    axes[0].plot(t_clip, free['gz_deg'][mask], label='ωz', color='tab:green')
    axes[0].set_ylabel('Gyro (°/s)')
    axes[0].legend(); axes[0].grid(True)

    axes[1].plot(t_clip, free['ax'][mask], label='ax', color='tab:blue')
    axes[1].plot(t_clip, free['ay'][mask], label='ay', color='tab:orange')
    axes[1].plot(t_clip, free['az'][mask], label='az', color='tab:green')
    axes[1].set_ylabel('Accel (m/s²)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(); axes[1].grid(True)

    plt.tight_layout()
    plt.savefig(fname, dpi=150)
    plt.show()

print('\nAll figures saved!')