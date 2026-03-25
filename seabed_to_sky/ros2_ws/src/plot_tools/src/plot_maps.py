#!/usr/bin/env python3
"""
plot_maps.py
------------
Loads sonar_map.pcd and bathy_map.pcd from saved_maps/PCD/ and generates:

  Static figures (saved to ../results/):
    uncertainty_map.png   — Bathymetry depth uncertainty (σ_z)
    semantic_map.png      — Top-down XY view, coloured by semantic class
    acoustic_surface_map.mat — Scattered X,Y,Z,Z_std for MATLAB scatter3()

  Interactive figure (saved to ../results/):
    sonar_map_3d.html     — 3-D point cloud coloured by semantic, opens in browser

Semantic classes:
  0.0 = SEABED     (coarse voxels, probability from Gaussian CDF)
  1.0 = STRUCTURE  (fine voxels, LiDAR-confirmed)
  2.0 = OBJECT     (fine voxels, sonar-only)

Run:
    python3 plot_maps.py
"""

import os
import sys
import struct

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import savemat
from sklearn.cluster import DBSCAN

try:
    import plotly.graph_objects as go
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False
    print('WARNING: plotly not installed — interactive figure will be skipped.')
    print('         Install with:  pip install plotly')

# ── Paths ─────────────────────────────────────────────────────────────────────
PCD_DIR     = '/home/rosdev/ros2_ws/saved_maps/PCD'
RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'results')
SONAR_PCD   = os.path.join(PCD_DIR, 'sonar_map.pcd')
BATHY_PCD   = os.path.join(PCD_DIR, 'bathy_map.pcd')

# ── DBSCAN parameters ─────────────────────────────────────────────────────────
DBSCAN_EPS         = 0.3
DBSCAN_MIN_SAMPLES = 3
OBJECT_MIN_VOXELS  = 10

# ── Grid resolution for interpolated maps ────────────────────────────────────
GRID_RES = 0.25     # [m]

# ── Publication style ─────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':        'serif',
    'font.size':          11,
    'axes.labelsize':     12,
    'axes.titlesize':     13,
    'xtick.labelsize':    10,
    'ytick.labelsize':    10,
    'legend.fontsize':    10,
    'figure.dpi':         150,
    'savefig.dpi':        300,
    'savefig.bbox':       'tight',
    'axes.spines.top':    False,
    'axes.spines.right':  False,
    'axes.grid':          False,
})


# ── PCD reader ────────────────────────────────────────────────────────────────

def read_pcd(path, label):
    """Read a binary float32 PCD file. Returns (N, F) float32 array."""
    if not os.path.isfile(path):
        sys.exit(f'ERROR: {label} not found at {path}')

    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.upper().startswith('DATA'):
                data_type = line.split()[1].lower()
                break
            if line and not line.startswith('#'):
                key, *vals = line.split()
                header[key.upper()] = vals
        data_offset = f.tell()
        raw = f.read()

    n_points = int(header['POINTS'][0])
    n_fields = len(header['FIELDS'])

    if data_type == 'binary':
        arr = np.frombuffer(raw, dtype=np.float32)
        arr = arr[: n_points * n_fields].reshape(n_points, n_fields)
    else:
        arr = np.array([list(map(float, l.split())) for l in raw.decode().splitlines()
                        if l.strip()], dtype=np.float32)

    print(f'{label}: {arr.shape[0]:,} points, fields: {header["FIELDS"]}')
    return arr, header['FIELDS']


# ── helpers ───────────────────────────────────────────────────────────────────

def make_grid(x, y, z, res=GRID_RES):
    xi = np.arange(x.min(), x.max() + res, res)
    yi = np.arange(y.min(), y.max() + res, res)
    Xi, Yi = np.meshgrid(xi, yi)
    Zi = np.full(Xi.shape, np.nan)
    ix = np.clip(np.round((x - xi[0]) / res).astype(int), 0, len(xi) - 1)
    iy = np.clip(np.round((y - yi[0]) / res).astype(int), 0, len(yi) - 1)
    Zi[iy, ix] = z
    return Xi, Yi, Zi


def save_fig(fig, name):
    out = os.path.join(RESULTS_DIR, name)
    fig.savefig(out, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {out}')


# ── MATLAB export ─────────────────────────────────────────────────────────────

def export_matlab(bathy):
    """Export visited bathy cells as scattered X, Y, Z, Z_std vectors.

        MATLAB usage:
        d = load('acoustic_surface_map.mat');
        scatter3(d.X, d.Y, d.Z, 10, d.Z, 'filled');
        colormap(flipud(parula)); colorbar;
        xlabel('East [m]'); ylabel('North [m]'); zlabel('Depth [m]');
        axis equal;
    """
    x, y, mu_z, std_z = bathy[:, 0], bathy[:, 1], bathy[:, 2], bathy[:, 3]
    out = os.path.join(RESULTS_DIR, 'acoustic_surface_map.mat')
    savemat(out, {
        'X':     x.reshape(-1, 1),
        'Y':     y.reshape(-1, 1),
        'Z':     mu_z.reshape(-1, 1),
        'Z_std': std_z.reshape(-1, 1),
    })
    print(f'MATLAB export: {out}  ({len(x):,} visited cells)')


# ── Static figure: depth uncertainty ─────────────────────────────────────────

def plot_uncertainty(bathy):
    x, y, mu_z, std_z = bathy[:, 0], bathy[:, 1], bathy[:, 2], bathy[:, 3]
    fig, ax = plt.subplots(figsize=(10, 8))
    sc = ax.scatter(x, y, c=std_z, cmap='hot_r', s=60, linewidths=0, alpha=0.9)
    cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('Depth uncertainty  σ [m]', labelpad=8)
    ax.set_xlabel('Easting  [m]')
    ax.set_ylabel('Northing  [m]')
    ax.set_title('Bathymetry depth uncertainty (visited regions only)')
    ax.set_aspect('equal')
    fig.tight_layout()
    save_fig(fig, 'uncertainty_map.png')


# ── Static figure: top-down semantic map ─────────────────────────────────────

def plot_semantic(voxels, bathy):
    # sonar_map.pcd fields: x y z probability intensity semantic
    semantic = voxels[:, 5]
    seabed_m = semantic == 0.0
    struct_m = semantic == 1.0
    object_m = semantic == 2.0

    fig, ax = plt.subplots(figsize=(7, 6.5))

    if bathy.shape[0] > 0:
        bx, by, bz = bathy[:, 0], bathy[:, 1], bathy[:, 2]
        Xi, Yi, Zi = make_grid(bx, by, bz)
        ax.pcolormesh(Xi, Yi, Zi, cmap='Greys', shading='auto', alpha=0.45, zorder=0)

    if np.any(struct_m):
        ax.scatter(voxels[struct_m, 0], voxels[struct_m, 1],
                   c='#D62728', s=18, linewidths=0,
                   label=f'Structure  (N={struct_m.sum():,})', zorder=3)

    n_obj_clusters = 0
    if np.any(object_m):
        obj_xy = voxels[object_m, :2]
        db     = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(obj_xy)
        labels = db.labels_
        plotted_any = False
        for lbl in np.unique(labels):
            if lbl == -1:
                continue
            mask = labels == lbl
            if mask.sum() < OBJECT_MIN_VOXELS:
                continue
            label_str = f'Object clusters  (≥{OBJECT_MIN_VOXELS} voxels)' \
                        if not plotted_any else '_nolegend_'
            ax.scatter(obj_xy[mask, 0], obj_xy[mask, 1],
                       c='#2CA02C', s=25, linewidths=0, label=label_str, zorder=4)
            ax.plot(obj_xy[mask, 0].mean(), obj_xy[mask, 1].mean(),
                    'k+', markersize=6, zorder=5)
            n_obj_clusters += 1
            plotted_any = True

    ax.set_xlabel('Easting  [m]')
    ax.set_ylabel('Northing  [m]')
    ax.set_title(
        f'Structures and objects of interest\n'
        f'Structures: {struct_m.sum():,} voxels  |  Object clusters: {n_obj_clusters}')
    ax.set_aspect('equal')
    if struct_m.any() or n_obj_clusters > 0:
        ax.legend(loc='upper right', framealpha=0.85, edgecolor='none')
    fig.tight_layout()
    save_fig(fig, 'semantic_map.png')


# ── Interactive 3-D figure ────────────────────────────────────────────────────

def plot_interactive_3d(voxels, bathy):
    if not HAS_PLOTLY:
        return

    # sonar_map.pcd fields: x y z probability intensity semantic
    x, y, z     = voxels[:, 0], voxels[:, 1], voxels[:, 2]
    probability = voxels[:, 3]
    intensity   = voxels[:, 4]
    semantic    = voxels[:, 5]

    CLASS_COLORS = {0.0: '#4C72B0', 1.0: '#D62728', 2.0: '#2CA02C'}
    CLASS_NAMES  = {0.0: 'Seabed', 1.0: 'Structure', 2.0: 'Object'}

    traces = []
    for cls in [0.0, 1.0, 2.0]:
        m = semantic == cls
        if not np.any(m):
            continue
        traces.append(go.Scatter3d(
            x=x[m], y=y[m], z=z[m],
            mode='markers',
            name=CLASS_NAMES[cls],
            marker=dict(
                size=2,
                color=CLASS_COLORS[cls],
                opacity=0.7,
            ),
            customdata=np.column_stack([probability[m], intensity[m]]),
            hovertemplate=(
                'x: %{x:.2f}  y: %{y:.2f}  z: %{z:.2f}<br>'
                'probability: %{customdata[0]:.3f}<br>'
                'intensity: %{customdata[1]:.0f}<extra>' + CLASS_NAMES[cls] + '</extra>'
            ),
        ))

    # Bathymetry surface (bathy_map.pcd: x y z_mu z_std hits)
    if bathy.shape[0] > 0:
        bx, by, bz = bathy[:, 0], bathy[:, 1], bathy[:, 2]
        Xi, Yi, Zi = make_grid(bx, by, bz)
        traces.append(go.Surface(
            x=Xi, y=Yi, z=Zi,
            colorscale='Blues',
            opacity=0.4,
            showscale=False,
            name='Bathy surface',
            hovertemplate='x: %{x:.2f}  y: %{y:.2f}  z: %{z:.2f}<extra>Bathy</extra>',
        ))

    fig = go.Figure(data=traces)
    fig.update_layout(
        title='Sonar map — 3D semantic view',
        scene=dict(
            xaxis_title='Easting [m]',
            yaxis_title='Northing [m]',
            zaxis_title='Z [m]',
            aspectmode='data',
        ),
        legend=dict(itemsizing='constant'),
        margin=dict(l=0, r=0, b=0, t=40),
    )

    out = os.path.join(RESULTS_DIR, 'sonar_map_3d.html')
    fig.write_html(out, include_plotlyjs='cdn')
    print(f'Saved interactive figure: {out}')


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    voxels, _ = read_pcd(SONAR_PCD, 'sonar_map.pcd')
    bathy,  _ = read_pcd(BATHY_PCD, 'bathy_map.pcd')

    semantic = voxels[:, 5]
    print(f'  seabed    : {(semantic == 0.0).sum():,}')
    print(f'  structure : {(semantic == 1.0).sum():,}')
    print(f'  object    : {(semantic == 2.0).sum():,}')

    export_matlab(bathy)
    plot_uncertainty(bathy)
    plot_semantic(voxels, bathy)
    plot_interactive_3d(voxels, bathy)

    print('Done.')


if __name__ == '__main__':
    main()
