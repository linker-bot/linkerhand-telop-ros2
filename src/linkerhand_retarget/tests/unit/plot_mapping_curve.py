#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mapping Curve Visualization Script
Plot open → opose → fist sensor and motor value curves for a single DOF

Usage:
    python3 plot_mapping_curve.py <joint_index>
    
    joint_index: 0-19 (robot_idx)
    
    Joint mapping:
        0: Thumb Rotate
        1: Thumb Abduction
        2: Thumb Root Flexion
        3: Thumb End Flexion
        5: Index Roll
        6: Index Root Flexion
        7: Index End Flexion
        9: Middle Roll
        10: Middle Root Flexion
        11: Middle End Flexion
        13: Ring Roll
        14: Ring Root Flexion
        15: Ring End Flexion
        17: Pinky Roll
        18: Pinky Root Flexion
        19: Pinky End Flexion
    
Example:
    python3 plot_mapping_curve.py 6  # Index Root Flexion
"""

import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Joint name mapping
JOINT_NAMES = {
    0: 'Thumb Rotate',
    1: 'Thumb Abduction',
    2: 'Thumb Root Flexion',
    3: 'Thumb End Flexion',
    5: 'Index Roll',
    6: 'Index Root Flexion',
    7: 'Index End Flexion',
    9: 'Middle Roll',
    10: 'Middle Root Flexion',
    11: 'Middle End Flexion',
    13: 'Ring Roll',
    14: 'Ring Root Flexion',
    15: 'Ring End Flexion',
    17: 'Pinky Roll',
    18: 'Pinky Root Flexion',
    19: 'Pinky End Flexion',
}

# Sensor index mapping (sensor array index for each robot_idx)
SENSOR_MAP = {
    0: 1,   # Thumb Rotate -> sensor[1]
    1: 0,   # Thumb Abduction -> sensor[0]
    2: 2,   # Thumb Root Flexion -> sensor[2]
    3: 4,   # Thumb End Flexion -> sensor[4]
    5: 5,   # Index Roll -> sensor[5]
    6: 6,   # Index Root Flexion -> sensor[6]
    7: 8,   # Index End Flexion -> sensor[8]
    9: 9,   # Middle Roll -> sensor[9]
    10: 10, # Middle Root Flexion -> sensor[10]
    11: 12, # Middle End Flexion -> sensor[12]
    13: 12, # Ring Roll -> sensor[12] (shared with middle)
    14: 14, # Ring Root Flexion -> sensor[14]
    15: 16, # Ring End Flexion -> sensor[16]
    17: 17, # Pinky Roll -> sensor[17]
    18: 18, # Pinky Root Flexion -> sensor[18]
    19: 20, # Pinky End Flexion -> sensor[20]
}

# exp_factor for each joint
EXP_FACTORS = {
    0: 1,   # Thumb Rotate
    1: 1,   # Thumb Abduction
    2: 5,   # Thumb Root Flexion
    3: 7,   # Thumb End Flexion
    5: 1,   # Index Roll
    6: 4,   # Index Root Flexion
    7: 3,   # Index End Flexion
    9: 1,   # Middle Roll
    10: 5,  # Middle Root Flexion
    11: 10, # Middle End Flexion
    13: 1,  # Ring Roll
    14: 5,  # Ring Root Flexion
    15: 18, # Ring End Flexion
    17: 1,  # Pinky Roll
    18: 5,  # Pinky Root Flexion
    19: 8,  # Pinky End Flexion
}

TMP_FILE = Path(__file__).resolve().parent.parent.parent / "linkerhand_retarget" / "motion" / "linkerforce" / "tmp" / "jointangle_data.tmp"

MOTOR_OPEN = 255
MOTOR_OPOSE = 128
MOTOR_FIST = 0

def map_value(sensor_val, sensor_open, sensor_opose, sensor_fist, exp_factor, debug=False):
    if abs(sensor_opose - sensor_open) < 1e-6:
        normalized = 0.5
    else:
        normalized = (sensor_val - sensor_open) / (sensor_opose - sensor_open)
    
    if normalized <= 0:
        return MOTOR_OPEN, normalized
    elif normalized <= 1:
        return MOTOR_OPEN + normalized * (MOTOR_OPOSE - MOTOR_OPEN), normalized
    else:
        normalized_fist = (sensor_fist - sensor_open) / (sensor_opose - sensor_open) if abs(sensor_opose - sensor_open) > 1e-6 else 1.5
        t_max = normalized_fist - 1.0
        t = min(normalized - 1.0, t_max)
        slope = MOTOR_FIST - MOTOR_OPOSE
        S1 = MOTOR_OPOSE - MOTOR_OPEN
        k = slope / (t_max * S1) - 1
        ratio = t / t_max
        extension = slope * (ratio + k * ratio ** exp_factor) / (1 + k)
        result = MOTOR_OPOSE + extension
        if debug:
            print(f"normalized_fist={normalized_fist:.4f}, t_max={t_max:.4f}, slope={slope}, S1={S1}, k={k:.4f}, exp_factor={exp_factor}")
        return max(MOTOR_FIST, result), normalized

def generate_curve_data(sensor_open, sensor_opose, sensor_fist, exp_factor, steps=200):
    sensor_min = min(sensor_open, sensor_fist) - 0.1
    sensor_max = max(sensor_open, sensor_opose, sensor_fist) + 0.2
    
    sensor_values = np.linspace(sensor_min, sensor_max, steps)
    motor_values = []
    normalized_values = []
    
    for s in sensor_values:
        motor, normalized = map_value(s, sensor_open, sensor_opose, sensor_fist, exp_factor)
        motor_values.append(motor)
        normalized_values.append(normalized)
    
    return sensor_values, motor_values, normalized_values

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    
    try:
        joint_idx = int(sys.argv[1])
    except ValueError:
        print(f"Error: joint_index must be an integer")
        print(__doc__)
        return
    
    if joint_idx not in JOINT_NAMES:
        print(f"Error: joint_index {joint_idx} not found")
        print("Valid indices:", sorted(JOINT_NAMES.keys()))
        return
    
    sensor_idx = SENSOR_MAP.get(joint_idx, joint_idx)
    exp_factor = EXP_FACTORS.get(joint_idx, 1)
    joint_name = JOINT_NAMES[joint_idx]
    
    # Load calibration data
    with open(TMP_FILE) as f:
        data = json.load(f)
    
    open_r = data['jointangleoriginal_r']
    opose_r = data['jointangleopose_r']
    fist_r = data['jointanglefist_r']
    
    if sensor_idx >= len(open_r):
        print(f"Error: sensor_idx {sensor_idx} out of range")
        return
    
    sensor_open = open_r[sensor_idx]
    sensor_opose = opose_r[sensor_idx]
    sensor_fist = fist_r[sensor_idx]
    
    # Generate curve data
    sensor_vals, motor_vals, normalized_vals = generate_curve_data(sensor_open, sensor_opose, sensor_fist, exp_factor)
    
    # Print parameters
    map_value(sensor_opose + 0.01, sensor_open, sensor_opose, sensor_fist, exp_factor, debug=True)
    
    # Verify motor at normalized=1.2
    sensor_1_2 = sensor_open + 1.2 * (sensor_opose - sensor_open)
    motor_1_2, _ = map_value(sensor_1_2, sensor_open, sensor_opose, sensor_fist, exp_factor)
    print(f"motor at normalized=1.2: {motor_1_2:.1f}")
    
    # Create plot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax2 = ax.twinx()
    
    line1, = ax.plot(normalized_vals, motor_vals, color='#4ECDC4', linewidth=2.5, label='Motor Value')
    line2, = ax2.plot(normalized_vals, sensor_vals, color='gray', linewidth=1.5, linestyle='--', label='Sensor Value')
    
    ax.axvline(x=0, color='green', linestyle=':', alpha=0.7, linewidth=1.5, label='open (normalized=0)')
    ax.axvline(x=1, color='orange', linestyle=':', alpha=0.7, linewidth=1.5, label='opose (normalized=1)')
    
    normalized_fist = (sensor_fist - sensor_open) / (sensor_opose - sensor_open) if abs(sensor_opose - sensor_open) > 1e-6 else 0.5
    if normalized_fist > 1:
        ax.axvline(x=normalized_fist, color='red', linestyle=':', alpha=0.7, linewidth=1.5, label=f'fist (normalized={normalized_fist:.2f})')
    
    ax.axhline(y=MOTOR_OPEN, color='green', linestyle=':', alpha=0.3)
    ax.axhline(y=MOTOR_OPOSE, color='orange', linestyle=':', alpha=0.3)
    ax.axhline(y=MOTOR_FIST, color='red', linestyle=':', alpha=0.3)
    
    ax.scatter([0, 1], [MOTOR_OPEN, MOTOR_OPOSE], color='black', s=80, zorder=5)
    if normalized_fist > 1:
        motor_at_fist, _ = map_value(sensor_fist, sensor_open, sensor_opose, sensor_fist, exp_factor)
        ax.scatter([normalized_fist], [motor_at_fist], color='red', s=100, zorder=5, marker='*')
    
    ax.set_xlabel('Normalized Sensor Value', fontsize=11)
    ax.set_ylabel('Motor Value', fontsize=11)
    ax2.set_ylabel('Sensor Raw Value', fontsize=11)
    ax.set_title(f'{joint_name} (robot_idx={joint_idx}, exp_factor={exp_factor})', fontsize=13, fontweight='bold')
    
    lines = [line1, line2]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='upper right', fontsize=9)
    
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-20, 280)
    
    plt.tight_layout()
    
    output_path = Path(__file__).parent / "images" / f"mapping_curve_joint_{joint_idx}.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Chart saved to: {output_path}")
    plt.close()

if __name__ == "__main__":
    main()