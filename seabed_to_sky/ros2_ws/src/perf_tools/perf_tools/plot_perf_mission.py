#!/usr/bin/env python3
"""
plot_perf_mission.py
--------------------
End-of-mission plots that reuse analyze_perf.py's loaders/stats and OVERLAY the
mission_supervisor save/prune events (from mission/events.csv) so the
"latency climbs as the map grows, then drops after a save/prune" pattern is visible.

Produces in <mission-dir>/plots/:
  mission_latency.png    sky/seabed rolling-p95 latency vs time + event markers
  mission_resources.png  CPU% and RAM(MB) vs time + event markers
  mission_mapsize.png    map point_count vs time (per topic) + event markers

All three share ONE mission clock (messages.csv arrival_monotonic_ns min as t0);
events.csv monotonic_ns is on the same system-wide CLOCK_MONOTONIC, so the markers
line up with the curves.

Usage:
  python3 plot_perf_mission.py --run-dir <mission>/perf_data/<run_id> --mission-dir <mission>
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_perf as ap  # noqa: E402  (reuse loaders/stats)

# Lidar + sonar are pruned/saved TOGETHER on one RAM trigger, so every fired action
# collapses to a SINGLE "prune" line per event time across all plots. (lidar_prune /
# sonar_save_reset are the legacy split actions; 'prune' is the new unified one.)
_PRUNE_COLOR = 'tab:red'
_FIRED = {'prune', 'lidar_prune', 'sonar_save_reset'}


def _load_events(mission_dir):
    path = os.path.join(mission_dir, 'events.csv')
    if not os.path.isfile(path):
        return pd.DataFrame()
    e = pd.read_csv(path)
    for c in ('wall_ns', 'monotonic_ns'):
        if c in e:
            e[c] = pd.to_numeric(e[c], errors='coerce')
    return e


def _overlay_events(ax, events, t0_ns):
    """Draw ONE red 'prune' line per trigger. Only fired actions count (skipped/no-srv/
    error rows are ignored), and near-simultaneous lidar+sonar rows are de-duplicated by
    time so each prune shows as a single line."""
    if events is None or events.empty:
        return
    drawn = False
    seen = set()
    for _, row in events.iterrows():
        if str(row.get('action', '')) not in _FIRED:        # ignore *_SKIPPED/_NOSRV/_ERR
            continue
        mono = row.get('monotonic_ns', np.nan)
        if not np.isfinite(mono):
            continue
        x = (mono - t0_ns) / 1e9
        key = round(x, 1)                                    # collapse simultaneous prunes
        if key in seen:
            continue
        seen.add(key)
        ax.axvline(x, color=_PRUNE_COLOR, lw=1.3, ls='-', alpha=0.7,
                   label=('prune' if not drawn else None))
        drawn = True


def main(argv=None):
    pa = argparse.ArgumentParser(description=__doc__)
    pa.add_argument('--run-dir', required=True, help='perf_data/<run_id> with messages/resources/map_growth csv')
    pa.add_argument('--mission-dir', required=True, help='mission/ root (events.csv + plots/ output)')
    pa.add_argument('--config', default=None, help='perf_tools config.yaml')
    a = pa.parse_args(argv)

    cfg = ap.load_config(a.config)
    try:
        ap.setup_mpl(cfg)
    except Exception:
        pass

    window = float(cfg.get('plot', {}).get('rolling_window_s', 60))
    df = ap.load_messages(a.run_dir)
    t0 = float(df['arrival_monotonic_ns'].min())
    events = _load_events(a.mission_dir)
    plot_dir = os.path.join(a.mission_dir, 'plots')
    os.makedirs(plot_dir, exist_ok=True)

    # 1) pipeline latency (rolling p95) + the single prune event line.
    #    lidar pipeline = /ouster/points -> /cloud_registered  (FAST-LIO: scan in until the
    #                     EKF updates odometry + the map), stage B.
    #    sonar pipeline = /blueview_message_polar -> /sonar_map/last_insert_stamp  (sonar image
    #                     in until it is inserted into the map), stages C+D end-to-end.
    chains = {'lidar pipeline': ap.chain_latency(df, cfg, ['B']),
              'sonar pipeline': ap.chain_latency(df, cfg, ['C', 'D'])}
    fig, ax = plt.subplots(figsize=(11, 4))
    for name, d in chains.items():
        if d is None or d.empty or 'mission_s' not in d:
            continue
        t = d['mission_s'].to_numpy(dtype=float)
        v = d['latency_ms'].to_numpy(dtype=float)
        order = np.argsort(t)
        t, v = t[order], v[order]
        p95 = ap.rolling_by_time(t, v, window, lambda x: np.percentile(x, 95))
        ax.plot(t, p95, label=f'{name} p95')
    _overlay_events(ax, events, t0)
    ax.set_xlabel('mission time [s]'); ax.set_ylabel('latency [ms]')
    ax.set_title('Chain latency (rolling p95) with save/prune events')
    ax.legend(loc='upper left'); fig.tight_layout()
    fig.savefig(os.path.join(plot_dir, 'mission_latency.png'), dpi=150); plt.close(fig)

    # 2) resources (CPU%, RAM MB) + events
    r = ap._load_resources(a.run_dir)
    if r is not None and 'monotonic_ns' in r:
        rt = (pd.to_numeric(r['monotonic_ns'], errors='coerce') - t0) / 1e9
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        if 'cpu_total' in r:
            ax[0].plot(rt, r['cpu_total'], color='k'); ax[0].set_ylabel('CPU total %')
        if 'ram_used_mb' in r and 'ram_total_mb' in r:       # RAM% drives the prune trigger
            ram_pct = (pd.to_numeric(r['ram_used_mb'], errors='coerce')
                       / pd.to_numeric(r['ram_total_mb'], errors='coerce') * 100.0)
            ax[1].plot(rt, ram_pct, color='k'); ax[1].set_ylabel('RAM used [%]')
        elif 'ram_used_mb' in r:
            ax[1].plot(rt, r['ram_used_mb'], color='k'); ax[1].set_ylabel('RAM used [MB]')
        for x in ax:
            _overlay_events(x, events, t0)
        ax[0].set_title('Resources with save/prune events')
        ax[-1].set_xlabel('mission time [s]')
        h, l = ax[0].get_legend_handles_labels()
        if l:
            ax[0].legend(loc='upper left')
        fig.tight_layout()
        fig.savefig(os.path.join(plot_dir, 'mission_resources.png'), dpi=150); plt.close(fig)

    # 3) map size (point_count per topic) + events
    mg_path = os.path.join(a.run_dir, 'map_growth.csv')
    if os.path.isfile(mg_path):
        mg = pd.read_csv(mg_path)
        if 'arrival_monotonic_ns' in mg and 'point_count' in mg:
            fig, ax = plt.subplots(figsize=(11, 4))
            for topic, sub in mg.groupby('topic'):
                st = (pd.to_numeric(sub['arrival_monotonic_ns'], errors='coerce') - t0) / 1e9
                ax.plot(st, pd.to_numeric(sub['point_count'], errors='coerce'), label=str(topic))
            _overlay_events(ax, events, t0)
            ax.set_xlabel('mission time [s]'); ax.set_ylabel('map point_count')
            ax.set_title('Map growth with save/prune events')
            ax.legend(loc='upper left'); fig.tight_layout()
            fig.savefig(os.path.join(plot_dir, 'mission_mapsize.png'), dpi=150); plt.close(fig)

    print(f'[plot_perf_mission] wrote plots to {plot_dir}')


if __name__ == '__main__':
    main()
