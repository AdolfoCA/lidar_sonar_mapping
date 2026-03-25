#!/usr/bin/env python3
"""
voxel_count.py
--------------
Reads mission_log.csv and plots voxel count over time.

Run:
    python3 voxel_count.py
    python3 voxel_count.py /path/to/mission_log.csv
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Paths ─────────────────────────────────────────────────────────────────────
MISSION_DIR = '/home/rosdev/ros2_ws/saved_maps/mission'
RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'results')
LOG_FILE    = os.path.join(MISSION_DIR, 'mission_log.csv')
OUT_FILE    = os.path.join(RESULTS_DIR, 'voxel_count.png')
# ─────────────────────────────────────────────────────────────────────────────


def load_log(path: str) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=',', skip_header=1)
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return data  # columns: elapsed_s, scan_count, voxel_count


def plot_voxel_count(data: np.ndarray, out_path: str) -> None:
    elapsed = data[:, 0]
    voxels  = data[:, 2]

    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(elapsed, voxels, linewidth=1.5, color='steelblue')
    ax.fill_between(elapsed, voxels, alpha=0.15, color='steelblue')

    ax.set_xlabel('Elapsed time (s)', fontsize=12)
    ax.set_ylabel('Voxel count', fontsize=12)
    ax.set_title('Map growth over time', fontsize=13)

    ax.xaxis.set_major_locator(ticker.MultipleLocator(30))
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(
        lambda x, _: f'{int(x):,}'))

    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(axis='y', linestyle='--', alpha=0.4)
    ax.grid(axis='x', linestyle=':', alpha=0.3)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f'Saved: {out_path}')


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else LOG_FILE
    if not os.path.isfile(log_path):
        print(f'ERROR: log file not found: {log_path}', file=sys.stderr)
        sys.exit(1)

    data = load_log(log_path)
    if data.shape[0] == 0:
        print('ERROR: log file is empty.', file=sys.stderr)
        sys.exit(1)

    plot_voxel_count(data, OUT_FILE)


if __name__ == '__main__':
    main()
