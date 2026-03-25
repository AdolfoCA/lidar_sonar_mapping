#!/usr/bin/env python3
"""
plot_mission_log.py
-------------------
Reads mission_log.csv and saves three separate figures:
  1. mission_intensity.png  — Seabed intensity estimate (mu_I ± std_I)
  2. mission_depth.png      — Seabed depth estimate    (mu_h ± std_h)
  3. mission_voxels.png     — Map growth over time (fine / seabed / total)

X-axis ticks every 30 seconds.
Figures are saved to ../results/  — no interactive window is opened.

Run:
    python3 plot_mission_log.py
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')   # non-interactive backend — no display needed
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Paths — edit if your setup differs ───────────────────────────────────────
MISSION_DIR  = '/home/rosdev/ros2_ws/saved_maps/mission'
RESULTS_DIR  = os.path.join(os.path.dirname(__file__), '..', 'results')
LOG_FILE     = os.path.join(MISSION_DIR, 'mission_log.csv')
# ─────────────────────────────────────────────────────────────────────────────


def _apply_time_axis(ax, t):
    ax.set_xlabel('Mission time [s]')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(30))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(10))
    ax.set_xlim(left=0)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    if not os.path.isfile(LOG_FILE):
        sys.exit(f'ERROR: mission log not found at {LOG_FILE}')

    try:
        data = np.loadtxt(LOG_FILE, delimiter=',', skiprows=1)
    except Exception as exc:
        sys.exit(f'ERROR loading {LOG_FILE}: {exc}')

    if data.ndim == 1:
        data = data[np.newaxis, :]
    if data.shape[0] == 0:
        sys.exit('ERROR: mission log is empty.')

    t     = data[:, 0] - data[0, 0]
    mu_I  = data[:, 1]
    std_I = data[:, 2]
    mu_h  = data[:, 3]
    std_h = data[:, 4]
    has_voxel_cols = data.shape[1] >= 7
    if has_voxel_cols:
        n_fine   = data[:, 5]
        n_seabed = data[:, 6]

    # ── Figure 1: Seabed intensity ────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t, mu_I, color='steelblue', linewidth=1.5, label='μ_I  (seabed intensity)')
    ax.fill_between(t, mu_I - std_I, mu_I + std_I,
                    alpha=0.25, color='steelblue', label='±σ_I')
    ax.set_ylabel('Intensity [counts]')
    ax.set_title('Seabed intensity estimate')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, which='major', alpha=0.5)
    ax.grid(True, which='minor', alpha=0.2)
    _apply_time_axis(ax, t)
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'mission_intensity.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {out}')

    # ── Figure 2: Seabed depth ────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t, mu_h, color='seagreen', linewidth=1.5, label='μ_h  (depth below USV)')
    ax.fill_between(t, mu_h - std_h, mu_h + std_h,
                    alpha=0.25, color='seagreen', label='±σ_h')
    ax.set_ylabel('Depth [m]')
    ax.set_title('Seabed depth estimate')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, which='major', alpha=0.5)
    ax.grid(True, which='minor', alpha=0.2)
    _apply_time_axis(ax, t)
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'mission_depth.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {out}')

    # ── Figure 3: Voxel growth ────────────────────────────────────────────────
    if has_voxel_cols:
        fig, ax = plt.subplots(figsize=(14, 4))
        ax.plot(t, n_fine,            color='darkorange',   linewidth=1.5,
                label='structure/object voxels (fine)')
        ax.plot(t, n_seabed,          color='mediumpurple', linewidth=1.5,
                label='seabed voxels (coarse)')
        ax.plot(t, n_fine + n_seabed, color='dimgray',      linewidth=1.5,
                linestyle='--', label='total voxels')
        ax.set_ylabel('Voxel count')
        ax.set_title('Map growth over time')
        ax.legend(loc='upper left', fontsize=9)
        ax.grid(True, which='major', alpha=0.5)
        ax.grid(True, which='minor', alpha=0.2)
        _apply_time_axis(ax, t)
        plt.tight_layout()
        out = os.path.join(RESULTS_DIR, 'mission_voxels.png')
        plt.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'Saved: {out}')


if __name__ == '__main__':
    main()
