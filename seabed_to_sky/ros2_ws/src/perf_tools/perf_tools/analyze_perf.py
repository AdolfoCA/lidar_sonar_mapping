#!/usr/bin/env python3
"""analyze_perf.py — offline analysis of a perf_tools capture run.

Runs on a laptop with ONLY pandas + matplotlib + numpy + pyyaml. Reads a run
folder produced by run_perf_capture.sh and emits publication-quality figures
(PDF + PNG) plus summary.csv for the paper's table.

    python3 analyze_perf.py --run-dir perf_data/20260615_120000 \
                            [--config config.yaml] [--warmup 60] \
                            [--plot-root plot]

Latency is derived OFFLINE by matching output messages to input messages on
header.stamp (exact after the FastLIO + seabed patches):
    A FastLIO odom    : /ouster/points        -> /odometry
    B FastLIO sky map : /ouster/points        -> /cloud_registered
    C leading edge    : /blueview_message_polar-> /blueview/point2/leading
    D seabed insert   : /blueview/point2/leading-> /sonar_map/last_insert_stamp
    chain sky    = B
    chain seabed = C + D (composed by stamp)
latency(stage) = arrival_monotonic(to) - arrival_monotonic(from) for matched stamp.
"""

from __future__ import annotations

import argparse
import os
import sys
import re

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    import yaml
except Exception:
    yaml = None


# --------------------------------------------------------------------------- #
# config / IO                                                                  #
# --------------------------------------------------------------------------- #
def _default_config_path() -> str:
    """Find config.yaml whether running from source or installed (ros2 run). The
    installed module sits in lib/.../site-packages while config.yaml installs to
    share/perf_tools/, so '../config.yaml' next to the module does NOT exist under
    a normal (non-symlink) install -- look in the ament share dir first."""
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('perf_tools'), 'config.yaml')
        if os.path.isfile(cand):
            return cand
    except Exception:                                   # noqa: BLE001 (ament not found)
        pass
    here = os.path.dirname(os.path.realpath(__file__))  # realpath -> resolve symlink-install
    for cand in (os.path.join(here, '..', 'config.yaml'),   # src/perf_tools/config.yaml
                 os.path.join(here, 'config.yaml')):
        if os.path.isfile(cand):
            return os.path.abspath(cand)
    return os.path.abspath(os.path.join(here, '..', 'config.yaml'))


def load_config(path: str | None) -> dict:
    if not path:
        path = _default_config_path()
    if yaml is None or not os.path.isfile(path):
        print(f'[analyze] WARNING: config not found at {path}; using built-in defaults',
              file=sys.stderr)
        return {}
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def role_topic_map(cfg: dict) -> dict:
    """role -> topic name."""
    return {t['role']: t['name'] for t in cfg.get('topics', []) if 'role' in t}


def setup_mpl(cfg: dict):
    plot = cfg.get('plot', {})
    plt.rcParams.update({
        'font.family': plot.get('font_family', 'serif'),
        'mathtext.fontset': 'cm',
        'font.size': 8,
        'axes.titlesize': 9,
        'axes.labelsize': 8,
        'legend.fontsize': 7,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'figure.dpi': 120,
        'savefig.dpi': plot.get('dpi', 300),
        'axes.grid': True,
        'grid.alpha': 0.3,
        'lines.linewidth': 1.0,
    })


# --------------------------------------------------------------------------- #
# data loading                                                                 #
# --------------------------------------------------------------------------- #
def load_messages(run_dir: str) -> pd.DataFrame:
    path = os.path.join(run_dir, 'messages.csv')
    df = pd.read_csv(path)
    # numeric coercion; blank header_stamp/point_count become NaN
    for col in ('header_stamp_ns', 'arrival_monotonic_ns', 'msg_size_bytes', 'point_count'):
        if col in df:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df


def mission_time(df: pd.DataFrame) -> pd.Series:
    """Seconds since first arrival across all topics (monotonic)."""
    t0 = df['arrival_monotonic_ns'].min()
    return (df['arrival_monotonic_ns'] - t0) / 1e9


# --------------------------------------------------------------------------- #
# latency matching                                                             #
# --------------------------------------------------------------------------- #
def match_stage(df: pd.DataFrame, from_topic: str, to_topic: str) -> pd.DataFrame:
    """Match output (to_topic) to input (from_topic) by header_stamp_ns.

    Returns DataFrame: stamp_ns, t_in_ns, t_out_ns, latency_ms, mission_s.
    """
    fin = df[(df['topic'] == from_topic) & df['header_stamp_ns'].notna()]
    fout = df[(df['topic'] == to_topic) & df['header_stamp_ns'].notna()]
    if fin.empty or fout.empty:
        return pd.DataFrame(columns=['stamp_ns', 't_in_ns', 't_out_ns',
                                     'latency_ms', 'mission_s'])
    # earliest arrival per stamp on each side (robust to duplicate stamps)
    a_in = fin.groupby('header_stamp_ns')['arrival_monotonic_ns'].min()
    a_out = fout.groupby('header_stamp_ns')['arrival_monotonic_ns'].min()
    joined = pd.concat([a_in.rename('t_in_ns'), a_out.rename('t_out_ns')],
                       axis=1, join='inner').reset_index()
    joined.rename(columns={'header_stamp_ns': 'stamp_ns'}, inplace=True)
    joined['latency_ms'] = (joined['t_out_ns'] - joined['t_in_ns']) / 1e6
    # drop nonsensical negatives (clock edge cases)
    joined = joined[joined['latency_ms'] >= 0].copy()
    t0 = df['arrival_monotonic_ns'].min()
    joined['mission_s'] = (joined['t_out_ns'] - t0) / 1e9
    return joined.sort_values('mission_s').reset_index(drop=True)


def compute_stages(df: pd.DataFrame, cfg: dict) -> dict:
    """Return {stage_key: matched DataFrame} for A,B,C,D and chains."""
    rt = role_topic_map(cfg)
    stages_cfg = cfg.get('stages', {})
    out = {}
    for key, sc in stages_cfg.items():
        ftop = rt.get(sc['from'])
        ttop = rt.get(sc['to'])
        if not ftop or not ttop:
            out[key] = pd.DataFrame(columns=['stamp_ns', 'latency_ms', 'mission_s'])
            continue
        out[key] = match_stage(df, ftop, ttop)
    return out


def chain_latency(df: pd.DataFrame, cfg: dict, stage_keys: list[str]) -> pd.DataFrame:
    """Compose an end-to-end chain by chaining stages on shared stamps.

    For the seabed chain [C, D] we match leading-edge by sonar stamp (C) then the
    insert by leading-edge stamp (D) — but both C and D share the SAME underlying
    sonar header.stamp (it is preserved through the chain), so end-to-end latency
    = arrival(final output) - arrival(original input) for each common stamp.
    """
    rt = role_topic_map(cfg)
    stages_cfg = cfg.get('stages', {})
    first_from = rt.get(stages_cfg[stage_keys[0]]['from'])
    last_to = rt.get(stages_cfg[stage_keys[-1]]['to'])
    if not first_from or not last_to:
        return pd.DataFrame(columns=['stamp_ns', 'latency_ms', 'mission_s'])
    return match_stage(df, first_from, last_to)


# --------------------------------------------------------------------------- #
# stats helpers                                                                #
# --------------------------------------------------------------------------- #
def pctl(s: pd.Series, q: float) -> float:
    return float(np.nanpercentile(s, q)) if len(s) else float('nan')


def warmup_filter(d: pd.DataFrame, warmup_s: float) -> pd.DataFrame:
    if d.empty or 'mission_s' not in d:
        return d
    return d[d['mission_s'] >= warmup_s].copy()


def rolling_by_time(t: np.ndarray, v: np.ndarray, window_s: float, func) -> np.ndarray:
    """Causal rolling stat over a time window (t sorted, seconds)."""
    out = np.full(len(t), np.nan)
    lo = 0
    for i in range(len(t)):
        while t[i] - t[lo] > window_s:
            lo += 1
        if i >= lo:
            out[i] = func(v[lo:i + 1])
    return out


def linfit_r2(x: np.ndarray, y: np.ndarray):
    """Return (slope, intercept, r2) for y ~ a*x + b."""
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 3 or np.ptp(x) == 0:
        return None
    a, b = np.polyfit(x, y, 1)
    yhat = a * x + b
    ss_res = np.sum((y - yhat) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return a, b, r2


# --------------------------------------------------------------------------- #
# figure savers                                                                #
# --------------------------------------------------------------------------- #
def save_fig(fig, plot_dir: str, name: str):
    os.makedirs(plot_dir, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(plot_dir, f'{name}.{ext}'), bbox_inches='tight')
    plt.close(fig)
    print(f'[analyze] wrote {name}.pdf / .png')


def _w(cfg, key, default):
    return cfg.get('plot', {}).get(key, default)


# --- individual figures ----------------------------------------------------- #
def fig_latency_cdf(chains, cfg, plot_dir):
    fig, ax = plt.subplots(figsize=(_w(cfg, 'single_col_in', 3.5), 2.6))
    any_data = False
    for label, d in chains.items():
        if d.empty:
            continue
        any_data = True
        v = np.sort(d['latency_ms'].values)
        y = np.arange(1, len(v) + 1) / len(v)
        ax.plot(v, y, label=label)
        med, p95, mx = np.median(v), np.percentile(v, 95), v.max()
        ax.annotate(f'{label}\n med {med:.1f} p95 {p95:.1f} max {mx:.1f} ms',
                    xy=(p95, 0.95), fontsize=6,
                    xytext=(0.45, 0.1 + 0.18 * list(chains).index(label)),
                    textcoords='axes fraction',
                    arrowprops=dict(arrowstyle='->', lw=0.5))
    ax.set_xlabel('End-to-end latency (ms)')
    ax.set_ylabel('CDF')
    ax.set_ylim(0, 1.02)
    if any_data:
        ax.legend(loc='lower right')
    ax.set_title('End-to-end latency CDF')
    save_fig(fig, plot_dir, 'latency_cdf')


def fig_stage_box(stages, cfg, plot_dir):
    fig, ax = plt.subplots(figsize=(_w(cfg, 'single_col_in', 3.5), 2.6))
    data, labels = [], []
    for key in ('A', 'B', 'C', 'D'):
        d = stages.get(key)
        if d is not None and not d.empty:
            data.append(d['latency_ms'].values)
            labels.append(f'{key}: {cfg["stages"][key]["name"]}')
    if data:
        ax.boxplot(data, showfliers=False)
        ax.set_xticks(range(1, len(labels) + 1))
        ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.set_ylabel('Stage latency (ms)')
    ax.set_title('Per-component processing latency')
    save_fig(fig, plot_dir, 'stage_latency_box')


def fig_stage_breakdown(stages, cfg, plot_dir):
    fig, ax = plt.subplots(figsize=(_w(cfg, 'single_col_in', 3.5), 2.6))
    # sky chain = [B]; seabed = [C, D]
    chains = {'LiDAR->sky': ['B'], 'Sonar->seabed': ['C', 'D']}
    bottoms = {}
    x = list(range(len(chains)))
    for i, (cname, keys) in enumerate(chains.items()):
        bottom = 0.0
        for key in keys:
            d = stages.get(key)
            med = float(np.median(d['latency_ms'])) if (d is not None and not d.empty) else 0.0
            ax.bar(i, med, bottom=bottom, label=f'{key}' if i == 0 or key not in bottoms else None)
            ax.text(i, bottom + med / 2, f'{key}\n{med:.1f}', ha='center', va='center', fontsize=6)
            bottom += med
            bottoms[key] = True
    ax.set_xticks(x)
    ax.set_xticklabels(list(chains.keys()))
    ax.set_ylabel('Median latency (ms)')
    ax.set_title('Where the time goes (median)')
    save_fig(fig, plot_dir, 'stage_breakdown')


def fig_degradation(chains, df, cfg, plot_dir):
    window = _w(cfg, 'rolling_window_s', 60)
    rt = role_topic_map(cfg)
    fig, ax = plt.subplots(figsize=(_w(cfg, 'double_col_in', 7.16), 3.0))
    for label, d in chains.items():
        if d.empty:
            continue
        t = d['mission_s'].values
        v = d['latency_ms'].values
        med = rolling_by_time(t, v, window, np.median)
        p95 = rolling_by_time(t, v, window, lambda a: np.percentile(a, 95))
        ax.plot(t, med, label=f'{label} median')
        ax.plot(t, p95, '--', label=f'{label} p95')
    ax.set_xlabel('Mission time (s)')
    ax.set_ylabel(f'Rolling latency (ms, {int(window)} s)')
    ax.set_title('Latency degradation over the mission')
    # twin axis: occupied seabed voxel/point count
    seabed = rt.get('seabed_map')
    if seabed:
        m = df[(df['topic'] == seabed) & df['point_count'].notna()]
        if not m.empty:
            t0 = df['arrival_monotonic_ns'].min()
            ax2 = ax.twinx()
            ax2.plot((m['arrival_monotonic_ns'] - t0) / 1e9, m['point_count'],
                     color='tab:gray', alpha=0.6, label='seabed points')
            ax2.set_ylabel('Seabed map points')
            ax2.grid(False)
    ax.legend(loc='upper left', ncol=2)
    save_fig(fig, plot_dir, 'degradation_over_time')


def fig_rtf(chains, cfg, plot_dir):
    """Real-time factor = end-to-end latency / sensor period (budget utilisation).

    RTF < 1 means the chain finishes a scan within ONE sensor period, i.e. it is
    done before the next scan arrives -> it keeps up in real time. This is the
    meaningful real-time metric for the embedded claim (the raw inter-arrival of
    a bag-replayed sensor is fixed by the replay, so it cannot show keep-up).
    The sensor period is taken from the FIRST stage's input topic nominal rate.
    """
    window = _w(cfg, 'rolling_window_s', 60)
    nominal = cfg.get('nominal_rates_hz', {})
    rt = role_topic_map(cfg)
    stages_cfg = cfg.get('stages', {})
    chains_cfg = cfg.get('chains', {})

    # map each chain (by display name) to the nominal period of its input sensor
    period_for = {}
    for cname, cc in chains_cfg.items():
        first_from_role = stages_cfg[cc['stages'][0]]['from']
        in_topic = rt.get(first_from_role)
        hz = nominal.get(in_topic)
        disp = cc.get('name', cname)
        period_for[disp] = (1.0 / hz, in_topic, hz) if hz else (None, in_topic, None)

    fig, ax = plt.subplots(figsize=(_w(cfg, 'double_col_in', 7.16), 3.0))
    first_cross_txt = []
    for label, d in chains.items():
        if d.empty or label not in period_for or period_for[label][0] is None:
            continue
        period_s, in_topic, hz = period_for[label]
        t = d['mission_s'].values
        rtf = (d['latency_ms'].values / 1e3) / period_s
        rtf_roll = rolling_by_time(t, rtf, window, np.median)
        ax.plot(t, rtf_roll, label=f'{label} (period {period_s*1e3:.0f} ms @ {hz:g} Hz)')
        # first sustained crossing ABOVE 1.0 (missing the deadline), 5 in a row
        above = rtf_roll > 1.0
        for i in range(len(above) - 5):
            if above[i:i + 5].all():
                ax.axvline(t[i], color='red', ls=':', lw=0.6)
                first_cross_txt.append(f'{label} @ {t[i]:.0f}s')
                break
    ax.axhline(1.0, color='k', lw=0.8)
    ax.set_ylim(bottom=0)
    ax.set_xlabel('Mission time (s)')
    ax.set_ylabel(f'Real-time factor = latency / sensor period\n(rolling {int(window)} s median)')
    title = 'Real-time factor (RTF<1 = keeps up within one sensor period)'
    if first_cross_txt:
        title += ' — first sustained RTF>1: ' + ', '.join(first_cross_txt)
    ax.set_title(title, fontsize=7)
    ax.legend(loc='best', ncol=1)
    save_fig(fig, plot_dir, 'rtf_over_time')


def _load_resources(run_dir):
    path = os.path.join(run_dir, 'resources.csv')
    if not os.path.isfile(path):
        return None
    r = pd.read_csv(path)
    if 'monotonic_ns' in r:
        r['t_s'] = (r['monotonic_ns'] - r['monotonic_ns'].min()) / 1e9
    return r


def fig_resources(run_dir, cfg, plot_dir):
    r = _load_resources(run_dir)
    if r is None or r.empty:
        print('[analyze] no resources.csv; skipping resources_timeseries')
        return
    throttle = cfg.get('throttle_temp_c', 85.0)
    proc_labels = [s['label'] for s in cfg.get('process_names', [])]
    temp_cols = [c for c in r.columns if c.startswith('temp_')]
    panels = ['CPU %', 'GPU %', 'RAM / RSS (MB)', 'Power (mW)', 'Temperature (C)']
    fig, axes = plt.subplots(len(panels), 1, sharex=True,
                             figsize=(_w(cfg, 'double_col_in', 7.16),
                                      _w(cfg, 'panel_height_in', 1.8) * len(panels)))
    t = r['t_s']
    # CPU total
    if 'cpu_total' in r:
        axes[0].plot(t, r['cpu_total'], color='tab:blue')
    axes[0].set_ylabel('CPU %')
    # GPU
    if 'gpu_pct' in r and r['gpu_pct'].notna().any():
        axes[1].plot(t, pd.to_numeric(r['gpu_pct'], errors='coerce'), color='tab:green')
    else:
        axes[1].text(0.5, 0.5, 'GPU N/A (PC)', transform=axes[1].transAxes,
                     ha='center', va='center', fontsize=7, color='gray')
    axes[1].set_ylabel('GPU %')
    # RAM + per-component RSS
    if 'ram_used_mb' in r:
        axes[2].plot(t, r['ram_used_mb'], label='RAM used', color='k')
    for label in proc_labels:
        col = f'proc_{label}_rss_mb'
        if col in r and pd.to_numeric(r[col], errors='coerce').notna().any():
            axes[2].plot(t, pd.to_numeric(r[col], errors='coerce'), label=label)
    axes[2].set_ylabel('MB')
    axes[2].legend(loc='upper left', ncol=2, fontsize=6)
    # Power
    if 'power_mw' in r and pd.to_numeric(r['power_mw'], errors='coerce').notna().any():
        axes[3].plot(t, pd.to_numeric(r['power_mw'], errors='coerce'), color='tab:red')
    else:
        axes[3].text(0.5, 0.5, 'Power N/A (PC)', transform=axes[3].transAxes,
                     ha='center', va='center', fontsize=7, color='gray')
    axes[3].set_ylabel('mW')
    # Temperature
    plotted_temp = False
    for c in temp_cols:
        vals = pd.to_numeric(r[c], errors='coerce')
        if vals.notna().any():
            axes[4].plot(t, vals, label=c.replace('temp_', ''))
            plotted_temp = True
    if plotted_temp:
        axes[4].axhline(throttle, color='red', ls='--', lw=0.8, label=f'throttle {throttle}C')
        axes[4].legend(loc='upper left', ncol=3, fontsize=6)
    else:
        axes[4].text(0.5, 0.5, 'Temp N/A', transform=axes[4].transAxes,
                     ha='center', va='center', fontsize=7, color='gray')
    axes[4].set_ylabel('C')
    axes[-1].set_xlabel('Mission time (s)')
    axes[0].set_title('System resources over the run')
    save_fig(fig, plot_dir, 'resources_timeseries')


def fig_cpu_per_component(run_dir, cfg, plot_dir):
    r = _load_resources(run_dir)
    if r is None or r.empty:
        return
    proc_labels = [s['label'] for s in cfg.get('process_names', [])]
    fig, ax = plt.subplots(figsize=(_w(cfg, 'double_col_in', 7.16), 2.6))
    t = r['t_s']
    plotted = False
    for label in proc_labels:
        col = f'proc_{label}_cpu'
        if col in r and pd.to_numeric(r[col], errors='coerce').notna().any():
            ax.plot(t, pd.to_numeric(r[col], errors='coerce'), label=label)
            plotted = True
    ax.set_xlabel('Mission time (s)')
    ax.set_ylabel('Process CPU %')
    ax.set_title('Per-component CPU usage')
    if plotted:
        ax.legend(loc='best')
    save_fig(fig, plot_dir, 'cpu_per_component')


def fig_insertion_vs_mapsize(stages, df, cfg, plot_dir):
    rt = role_topic_map(cfg)
    fig, axes = plt.subplots(1, 2, figsize=(_w(cfg, 'double_col_in', 7.16), 2.8))

    def scatter_fit(ax, x, y, xlabel, title):
        ax.scatter(x, y, s=4, alpha=0.4)
        ax.set_xlabel(xlabel)
        ax.set_ylabel('Latency (ms)')
        ax.set_title(title, fontsize=8)
        fit = linfit_r2(np.asarray(x, float), np.asarray(y, float))
        if fit:
            a, b, r2 = fit
            xs = np.linspace(np.nanmin(x), np.nanmax(x), 50)
            ax.plot(xs, a * xs + b, 'r-', lw=1.0, label=f'lin R²={r2:.2f}')
        # log fit y ~ a*ln(x)+b
        xa, ya = np.asarray(x, float), np.asarray(y, float)
        m = np.isfinite(xa) & np.isfinite(ya) & (xa > 0)
        if m.sum() >= 3 and np.ptp(np.log(xa[m])) > 0:
            la, lb = np.polyfit(np.log(xa[m]), ya[m], 1)
            yhat = la * np.log(xa[m]) + lb
            ss_res = np.sum((ya[m] - yhat) ** 2)
            ss_tot = np.sum((ya[m] - np.mean(ya[m])) ** 2)
            r2l = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
            xs = np.linspace(np.nanmin(xa[m]), np.nanmax(xa[m]), 50)
            ax.plot(xs, la * np.log(xs) + lb, 'g--', lw=1.0, label=f'log R²={r2l:.2f}')
        ax.legend(loc='best', fontsize=6)

    # D: stage-D latency vs seabed point count (match by output arrival time)
    d = stages.get('D')
    seabed = rt.get('seabed_map')
    if d is not None and not d.empty and seabed:
        smap = df[(df['topic'] == seabed) & df['point_count'].notna()].copy()
        if not smap.empty:
            smap = smap.sort_values('arrival_monotonic_ns')
            merged = pd.merge_asof(
                d.sort_values('t_out_ns'),
                smap[['arrival_monotonic_ns', 'point_count']].rename(
                    columns={'arrival_monotonic_ns': 't_out_ns'}),
                on='t_out_ns', direction='nearest')
            scatter_fit(axes[0], merged['point_count'], merged['latency_ms'],
                        'Seabed map points', 'Stage D vs seabed size')
        else:
            axes[0].set_title('Stage D: no seabed size data', fontsize=8)
    else:
        axes[0].set_title('Stage D: no data', fontsize=8)

    # B: stage-B latency vs sky-map (cloud_registered) point count, same stamp
    b = stages.get('B')
    sky = rt.get('fastlio_skymap')
    if b is not None and not b.empty and sky:
        skym = df[(df['topic'] == sky) & df['point_count'].notna()][
            ['header_stamp_ns', 'point_count']]
        bb = b.merge(skym, left_on='stamp_ns', right_on='header_stamp_ns', how='inner')
        if not bb.empty:
            scatter_fit(axes[1], bb['point_count'], bb['latency_ms'],
                        'Sky map points (per scan)', 'Stage B vs sky size')
        else:
            axes[1].set_title('Stage B: no joined size data', fontsize=8)
    else:
        axes[1].set_title('Stage B: no data', fontsize=8)
    save_fig(fig, plot_dir, 'insertion_vs_mapsize')


def parse_bag_counts(run_dir: str) -> dict:
    """Parse messages-per-topic from the dumped bag_info.txt."""
    path = os.path.join(run_dir, 'bag_info.txt')
    counts = {}
    if not os.path.isfile(path):
        return counts
    with open(path) as fh:
        text = fh.read()
    # lines like: "Topic: /ouster/points | Type: ... | Count: 36000 | ..."
    for m in re.finditer(r'Topic:\s*(\S+).*?Count:\s*(\d+)', text):
        counts[m.group(1)] = int(m.group(2))
    return counts


def fig_throughput_drops(df, cfg, run_dir, plot_dir):
    window = _w(cfg, 'rolling_window_s', 60)
    fig, ax = plt.subplots(figsize=(_w(cfg, 'double_col_in', 7.16), 3.0))
    t0 = df['arrival_monotonic_ns'].min()
    topics = [t['name'] for t in cfg.get('topics', [])]
    for topic in topics:
        m = df[df['topic'] == topic].sort_values('arrival_monotonic_ns')
        if len(m) < 3:
            continue
        ta = (m['arrival_monotonic_ns'].values - t0) / 1e9
        dt = np.diff(ta)
        hz = np.where(dt > 0, 1.0 / dt, np.nan)
        hz_roll = rolling_by_time(ta[1:], hz, window, np.nanmedian)
        ax.plot(ta[1:], hz_roll, label=topic)
    ax.set_xlabel('Mission time (s)')
    ax.set_ylabel(f'Achieved Hz (rolling {int(window)} s)')
    ax.set_title('Per-topic throughput')
    ax.legend(loc='best', ncol=2, fontsize=6)
    save_fig(fig, plot_dir, 'throughput_drops')


# --------------------------------------------------------------------------- #
# summary table                                                                #
# --------------------------------------------------------------------------- #
def build_summary(df, stages, chains, run_dir, cfg, warmup_s) -> pd.DataFrame:
    rows = []

    def lat_stats(name, d):
        if d is None or d.empty:
            rows.append({'metric': name, 'median_ms': '', 'p95_ms': '', 'max_ms': '',
                         'n': 0})
            return
        v = d['latency_ms']
        rows.append({'metric': name,
                     'median_ms': round(float(v.median()), 3),
                     'p95_ms': round(pctl(v, 95), 3),
                     'max_ms': round(float(v.max()), 3),
                     'n': int(len(v))})

    for key in ('A', 'B', 'C', 'D'):
        sc = cfg.get('stages', {}).get(key, {})
        lat_stats(f'stage_{key}_{sc.get("name", key)}', stages.get(key))
    for cname, d in chains.items():
        lat_stats(f'chain_{cname}', d)

    # throughput + drop rates
    bag_counts = parse_bag_counts(run_dir)
    for t in cfg.get('topics', []):
        topic = t['name']
        m = df[df['topic'] == topic]
        recv = len(m)
        achieved_hz = ''
        if recv > 2:
            span = (m['arrival_monotonic_ns'].max() - m['arrival_monotonic_ns'].min()) / 1e9
            achieved_hz = round(recv / span, 3) if span > 0 else ''
        bag_n = bag_counts.get(topic, '')
        drop = ''
        if isinstance(bag_n, int) and bag_n > 0:
            drop = round(100.0 * (1 - recv / bag_n), 2)
        rows.append({'metric': f'topic_{topic}', 'received': recv,
                     'in_bag': bag_n, 'drop_pct': drop, 'achieved_hz': achieved_hz})

    # resources: per-component CPU share + effective ms/msg, power, temp, energy
    r = _load_resources(run_dir)
    if r is not None and not r.empty:
        if 't_s' in r:
            r = r[r['t_s'] >= warmup_s]
        n_cores = max(1, len([c for c in r.columns if re.fullmatch(r'cpu\d+', c)]))
        for s in cfg.get('process_names', []):
            label = s['label']
            col = f'proc_{label}_cpu'
            if col in r:
                cpu = pd.to_numeric(r[col], errors='coerce')
                mean_cpu = float(cpu.mean()) if cpu.notna().any() else float('nan')
                # messages this component processed (its output topic count, post-warmup)
                rows.append({'metric': f'cpu_{label}',
                             'mean_cpu_pct': round(mean_cpu, 2),
                             'mean_cpu_share_pct': round(mean_cpu / n_cores, 2)})
        # power / temp / energy
        if 'power_mw' in r:
            pw = pd.to_numeric(r['power_mw'], errors='coerce')
            if pw.notna().any():
                mean_pw = float(pw.mean())
                dur = float(r['t_s'].max() - r['t_s'].min()) if 't_s' in r else 0.0
                energy_j = mean_pw / 1000.0 * dur
                rows.append({'metric': 'power', 'mean_power_mw': round(mean_pw, 1),
                             'total_energy_J': round(energy_j, 1)})
        temp_cols = [c for c in r.columns if c.startswith('temp_')]
        peak = float('nan')
        for c in temp_cols:
            vals = pd.to_numeric(r[c], errors='coerce')
            if vals.notna().any():
                peak = np.nanmax([peak, float(vals.max())])
        if np.isfinite(peak):
            rows.append({'metric': 'temperature', 'peak_temp_c': round(peak, 1)})

    # effective ms-per-message: CPU-time / messages processed, per component.
    # CPU-time(s) = mean_cpu% / 100 * duration. messages = output topic count.
    rt = role_topic_map(cfg)
    out_role = {'fastlio': 'fastlio_odom', 'leading_edge': 'leading_edge',
                'seabed_map': 'seabed_insert'}
    if r is not None and not r.empty and 't_s' in r:
        dur = float(r['t_s'].max() - r['t_s'].min())
        for s in cfg.get('process_names', []):
            label = s['label']
            col = f'proc_{label}_cpu'
            role = out_role.get(label)
            topic = rt.get(role) if role else None
            if col in r and topic and dur > 0:
                cpu = pd.to_numeric(r[col], errors='coerce')
                if cpu.notna().any():
                    cpu_time_s = cpu.mean() / 100.0 * dur
                    nmsg = len(df[(df['topic'] == topic)])
                    if nmsg > 0:
                        rows.append({'metric': f'eff_ms_per_msg_{label}',
                                     'ms_per_msg': round(cpu_time_s / nmsg * 1000.0, 3),
                                     'messages': nmsg})
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------- #
# main                                                                         #
# --------------------------------------------------------------------------- #
def main(argv=None):
    parser = argparse.ArgumentParser(description='Offline analysis of a perf capture run.')
    parser.add_argument('--run-dir', required=True,
                        help='Capture folder (perf_data/<run_id>) or a run id under --out-root')
    parser.add_argument('--config', default=None, help='Path to config.yaml')
    parser.add_argument('--warmup', type=float, default=None,
                        help='Warm-up seconds to discard (default: config run.warmup_s or 60)')
    parser.add_argument('--plot-root', default=None,
                        help='Root for plot output (default: <run-dir>/../../plot)')
    parser.add_argument('--out-root', default=None,
                        help='If --run-dir is a bare id, the perf_data root holding it')
    args = parser.parse_args(argv)

    run_dir = args.run_dir
    if not os.path.isdir(run_dir) and args.out_root:
        run_dir = os.path.join(args.out_root, args.run_dir)
    if not os.path.isdir(run_dir):
        print(f'[analyze] run dir not found: {run_dir}', file=sys.stderr)
        return 2

    cfg = load_config(args.config)
    setup_mpl(cfg)
    warmup_s = args.warmup if args.warmup is not None \
        else float(cfg.get('run', {}).get('warmup_s', 60))

    run_id = os.path.basename(os.path.normpath(run_dir))
    plot_root = args.plot_root or os.path.abspath(os.path.join(run_dir, '..', '..', 'plot'))
    plot_dir = os.path.join(plot_root, run_id)
    os.makedirs(plot_dir, exist_ok=True)

    df = load_messages(run_dir)
    print(f'[analyze] {len(df)} message rows; warm-up = {warmup_s}s; plots -> {plot_dir}')

    stages_raw = compute_stages(df, cfg)
    stages = {k: warmup_filter(v, warmup_s) for k, v in stages_raw.items()}

    chains_cfg = cfg.get('chains', {})
    chains = {}
    for cname, cc in chains_cfg.items():
        d = chain_latency(df, cfg, cc['stages'])
        chains[cc.get('name', cname)] = warmup_filter(d, warmup_s)

    # figures
    fig_latency_cdf(chains, cfg, plot_dir)
    fig_stage_box(stages, cfg, plot_dir)
    fig_stage_breakdown(stages, cfg, plot_dir)
    fig_degradation(chains, df, cfg, plot_dir)
    fig_rtf(chains, cfg, plot_dir)
    fig_resources(run_dir, cfg, plot_dir)
    fig_cpu_per_component(run_dir, cfg, plot_dir)
    fig_insertion_vs_mapsize(stages, df, cfg, plot_dir)
    fig_throughput_drops(df, cfg, run_dir, plot_dir)

    summary = build_summary(df, stages, chains, run_dir, cfg, warmup_s)
    summary_path = os.path.join(plot_dir, 'summary.csv')
    summary.to_csv(summary_path, index=False)
    print(f'[analyze] wrote {summary_path}')
    print(summary.to_string(index=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
