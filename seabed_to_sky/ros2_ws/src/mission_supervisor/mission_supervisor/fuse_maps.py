#!/usr/bin/env python3
"""
fuse_maps.py
============
Fuse all saved per-prune map chunks into single point clouds.

The lidar (FAST-LIO ikd-tree) chunks in mission/lidar/ and the sonar (non-semantic
map) chunks in mission/sonar/ are ALL already in the same world frame, anchored at the
initial pose -- so fusing is a pure CONCATENATION, no per-cloud transform needed.

Outputs (default <mission>/fused/):
  fused_lidar.pcd   every mission/lidar/*.pcd concatenated
  fused_sonar.pcd   every mission/sonar/*.pcd concatenated
  fused_all.pcd     lidar + sonar combined, with a 'source' scalar (0=lidar, 1=sonar)

Each output carries x,y,z + intensity. Reads ascii / binary / binary_compressed PCD via
open3d's tensor IO (which also preserves the intensity field).

--voxel M  downsample to a voxel grid of M metres (shrinks the cloud AND de-duplicates
           overlapping revisits / overlapping prune saves). 0 = off.

NOTE: this fuses only the SAVED chunks. Points still resident in the LIVE maps -- the
ikd-tree near the final pose, and the sonar buffer since the last save_and_reset -- are
not on disk yet. Trigger one final prune + save_and_reset (or lower the RAM threshold so
the supervisor fires once) before fusing if you want the complete map.
"""
import argparse
import glob
import os


def _read_xyzi(path):
    """PCD (any encoding) -> (xyz Nx3 float32, intensity N float32) via open3d tensor IO."""
    import numpy as np
    import open3d as o3d
    pc = o3d.t.io.read_point_cloud(path)
    attr = pc.point
    if 'positions' not in attr or attr['positions'].shape[0] == 0:
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.float32)
    xyz = attr['positions'].numpy().astype(np.float32)
    if 'intensity' in attr:
        inten = attr['intensity'].numpy().reshape(-1).astype(np.float32)
    else:
        inten = np.zeros((xyz.shape[0],), np.float32)
    return xyz, inten


def _fuse_dir(d, exclude=()):
    import numpy as np
    allfiles = sorted(glob.glob(os.path.join(d, '*.pcd')))
    files = [f for f in allfiles
             if not any(s in os.path.basename(f) for s in exclude)]
    skipped = [os.path.basename(f) for f in allfiles if f not in files]
    if skipped:
        print(f"  excluded {len(skipped)} chunk(s) from {os.path.basename(d)}: {skipped}")
    xyzs, intens, chunks = [], [], []
    for i, f in enumerate(files):
        xyz, inten = _read_xyzi(f)
        if xyz.shape[0]:
            xyzs.append(xyz)
            intens.append(inten)
            chunks.append(np.full(xyz.shape[0], i, np.float32))  # per-file ordinal
    if not xyzs:
        return (np.zeros((0, 3), np.float32), np.zeros((0,), np.float32),
                np.zeros((0,), np.float32), files)
    return np.concatenate(xyzs), np.concatenate(intens), np.concatenate(chunks), files


def _voxel(xyz, inten, size, extras=None):
    """Voxel downsample, averaging intensity (and any extra scalar fields) per voxel."""
    import open3d as o3d
    pc = o3d.t.geometry.PointCloud()
    pc.point['positions'] = o3d.core.Tensor(xyz)
    pc.point['intensity'] = o3d.core.Tensor(inten.reshape(-1, 1))
    for k, v in (extras or {}).items():
        pc.point[k] = o3d.core.Tensor(v.reshape(-1, 1))
    ds = pc.voxel_down_sample(float(size))
    out_extras = {k: ds.point[k].numpy().reshape(-1) for k in (extras or {})}
    return ds.point['positions'].numpy(), ds.point['intensity'].numpy().reshape(-1), out_extras


def _write(path, xyz, inten, extras=None, binary=True):
    import open3d as o3d
    pc = o3d.t.geometry.PointCloud()
    pc.point['positions'] = o3d.core.Tensor(xyz)
    pc.point['intensity'] = o3d.core.Tensor(inten.reshape(-1, 1))
    for k, v in (extras or {}).items():
        pc.point[k] = o3d.core.Tensor(v.reshape(-1, 1))
    o3d.t.io.write_point_cloud(path, pc, write_ascii=not binary)


def _bbox(xyz):
    if not xyz.shape[0]:
        return "empty"
    lo, hi = xyz.min(0), xyz.max(0)
    return (f"x[{lo[0]:.1f},{hi[0]:.1f}] y[{lo[1]:.1f},{hi[1]:.1f}] "
            f"z[{lo[2]:.1f},{hi[2]:.1f}] m")


def main(argv=None):
    import numpy as np
    pa = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    pa.add_argument('--mission-dir', default='/home/rosdev/ros2_ws/mission')
    pa.add_argument('--lidar-dir', default=None, help='default <mission>/lidar')
    pa.add_argument('--sonar-dir', default=None, help='default <mission>/sonar')
    pa.add_argument('--out-dir', default=None, help='default <mission>/fused')
    pa.add_argument('--voxel', type=float, default=0.0,
                    help='voxel size (m) to downsample + dedup; 0 = off')
    pa.add_argument('--ascii', action='store_true', help='write ASCII pcd (default binary)')
    pa.add_argument('--no-combined', action='store_true',
                    help='skip the combined lidar+sonar cloud')
    pa.add_argument('--tag-chunks', action='store_true',
                    help="add a 'chunk' scalar (per-file ordinal) so you can color-by-chunk "
                         "in Foxglove to spot a misplaced save chunk")
    pa.add_argument('--exclude', default='',
                    help="comma-separated substrings; any chunk file whose name contains one "
                         "is skipped (e.g. bad odometry chunks: 183147,183248,183349)")
    a = pa.parse_args(argv)
    exclude = [s.strip() for s in a.exclude.split(',') if s.strip()]

    lidar_dir = a.lidar_dir or os.path.join(a.mission_dir, 'lidar')
    sonar_dir = a.sonar_dir or os.path.join(a.mission_dir, 'sonar')
    out_dir = a.out_dir or os.path.join(a.mission_dir, 'fused')
    os.makedirs(out_dir, exist_ok=True)
    binary = not a.ascii

    lx, li, lc, lf = _fuse_dir(lidar_dir, exclude)
    sx, si, sc, sf = _fuse_dir(sonar_dir, exclude)
    print(f"lidar: {len(lf)} files -> {lx.shape[0]:,} pts  {_bbox(lx)}")
    print(f"sonar: {len(sf)} files -> {sx.shape[0]:,} pts  {_bbox(sx)}")
    if lx.shape[0] == 0 and sx.shape[0] == 0:
        print(f"nothing to fuse (no *.pcd in {lidar_dir} or {sonar_dir}). "
              f"Trigger a prune/save first.")
        return

    lextra = {'chunk': lc} if a.tag_chunks else {}
    sextra = {'chunk': sc} if a.tag_chunks else {}
    if a.voxel > 0:
        if lx.shape[0]:
            lx, li, lextra = _voxel(lx, li, a.voxel, lextra)
        if sx.shape[0]:
            sx, si, sextra = _voxel(sx, si, a.voxel, sextra)
        print(f"after voxel {a.voxel:g} m -> lidar {lx.shape[0]:,}  sonar {sx.shape[0]:,}")

    if lx.shape[0]:
        p = os.path.join(out_dir, 'fused_lidar.pcd')
        _write(p, lx, li, extras=lextra, binary=binary)
        print('wrote', p, f"(fields: x y z intensity{' chunk' if a.tag_chunks else ''})")
    if sx.shape[0]:
        p = os.path.join(out_dir, 'fused_sonar.pcd')
        _write(p, sx, si, extras=sextra, binary=binary)
        print('wrote', p)
    if not a.no_combined and (lx.shape[0] or sx.shape[0]):
        axyz = np.concatenate([lx, sx]) if (lx.shape[0] and sx.shape[0]) else (lx if lx.shape[0] else sx)
        ain = np.concatenate([li, si]) if (lx.shape[0] and sx.shape[0]) else (li if lx.shape[0] else si)
        src = np.concatenate([np.zeros(lx.shape[0], np.float32),
                              np.ones(sx.shape[0], np.float32)])
        p = os.path.join(out_dir, 'fused_all.pcd')
        _write(p, axyz, ain, extras={'source': src}, binary=binary)
        print(f"wrote {p}  ({axyz.shape[0]:,} pts; field 'source': 0=lidar 1=sonar)")
    print('done ->', out_dir)


if __name__ == '__main__':
    main()
