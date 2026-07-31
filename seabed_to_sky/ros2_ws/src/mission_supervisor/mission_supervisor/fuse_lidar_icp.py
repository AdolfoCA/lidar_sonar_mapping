#!/usr/bin/env python3
"""
fuse_lidar_icp.py
=================
Post-hoc ICP de-ghosting for the fused lidar map.

FAST-LIO has no loop closure, so over a long mission its pose drifts ~0.5-2 m. The
saved prune chunks are each internally fine, but the late (return-pass) chunks land
offset from the earlier map of the same structures -> double walls / ghosting when
naively concatenated.

This registers each chunk (chronologically) onto a GROWING reference map with
point-to-plane ICP and applies the correction, snapping the drifted return pass back
onto the outbound map. Chunk 0 is the anchor (kept fixed). Chunks with too little
overlap to trust (new far area) are left untouched (identity).

Outputs <out> (default mission/fused/fused_lidar_icp.pcd), x,y,z,intensity.

CAVEAT: each chunk gets ONE rigid correction, but a far-slab chunk mixes points
observed at different times (different drift), so this reduces -- not perfectly
eliminates -- ghosting. The only complete fix is loop closure in the SLAM itself.
"""
import argparse
import glob
import os


def _read_xyzi(path):
    import numpy as np
    import open3d as o3d
    a = o3d.t.io.read_point_cloud(path).point
    xyz = a['positions'].numpy().astype(np.float64)
    inten = (a['intensity'].numpy().reshape(-1).astype(np.float32)
             if 'intensity' in a else np.zeros(xyz.shape[0], np.float32))
    m = np.isfinite(xyz).all(1)
    return xyz[m], inten[m]


def _pcd(xyz):
    import open3d as o3d
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(xyz)
    return p


def main(argv=None):
    import numpy as np
    import open3d as o3d
    reg = o3d.pipelines.registration
    pa = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    pa.add_argument('--mission-dir', default='/home/rosdev/ros2_ws/mission')
    pa.add_argument('--lidar-dir', default=None)
    pa.add_argument('--out', default=None)
    pa.add_argument('--icp-voxel', type=float, default=0.3, help='downsample (m) used for ICP')
    pa.add_argument('--out-voxel', type=float, default=0.1, help='final output voxel (0=off)')
    pa.add_argument('--coarse', type=float, default=2.5, help='coarse ICP max corr dist (m)')
    pa.add_argument('--fine', type=float, default=0.8, help='fine ICP max corr dist (m)')
    pa.add_argument('--min-fitness', type=float, default=0.3,
                    help='min ICP fitness to TRUST the correction (else keep chunk as-is)')
    pa.add_argument('--max-shift', type=float, default=5.0,
                    help='reject an ICP correction that translates more than this (m)')
    a = pa.parse_args(argv)

    lidar_dir = a.lidar_dir or os.path.join(a.mission_dir, 'lidar')
    out = a.out or os.path.join(a.mission_dir, 'fused', 'fused_lidar_icp.pcd')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    files = sorted(glob.glob(os.path.join(lidar_dir, '*.pcd')))
    if not files:
        print(f'no chunks in {lidar_dir}')
        return
    v = a.icp_voxel
    nn = o3d.geometry.KDTreeSearchParamHybrid(radius=3 * v, max_nn=30)

    # anchor = chunk 0
    xyz0, in0 = _read_xyzi(files[0])
    aligned = [(xyz0, in0)]
    ref = _pcd(xyz0).voxel_down_sample(v)
    ref.estimate_normals(nn)
    print(f"[icp] anchor {os.path.basename(files[0])}: {xyz0.shape[0]:,} pts")

    for i in range(1, len(files)):
        xyz, inten = _read_xyzi(files[i])
        src = _pcd(xyz).voxel_down_sample(v)
        T = np.eye(4)
        fit = rmse = 0.0
        if len(src.points) > 50 and len(ref.points) > 50:
            r = reg.registration_icp(src, ref, a.coarse, T,
                                     reg.TransformationEstimationPointToPlane(),
                                     reg.ICPConvergenceCriteria(max_iteration=40))
            r = reg.registration_icp(src, ref, a.fine, r.transformation,
                                     reg.TransformationEstimationPointToPlane(),
                                     reg.ICPConvergenceCriteria(max_iteration=40))
            fit, rmse = r.fitness, r.inlier_rmse
            shift = float(np.linalg.norm(r.transformation[:3, 3]))
            if fit >= a.min_fitness and shift <= a.max_shift:
                T = r.transformation
            else:
                T = np.eye(4)                       # untrusted -> leave chunk where it is
        # apply correction to the FULL-res chunk
        h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
        xyz_a = (T @ h.T).T[:, :3]
        aligned.append((xyz_a, inten))
        corr = float(np.linalg.norm(T[:3, 3]))
        tag = f"corr={corr:.2f}m fit={fit:.2f} rmse={rmse:.2f}" if corr > 1e-6 else f"kept (fit={fit:.2f})"
        print(f"[icp] {i:>2}/{len(files)-1} {os.path.basename(files[i]):<30} {xyz.shape[0]:>8,} pts  {tag}")
        # grow reference with the aligned chunk, keep it bounded + re-normal
        src.transform(T)
        ref = (ref + src).voxel_down_sample(v)
        ref.estimate_normals(nn)

    # write fused aligned cloud
    allxyz = np.concatenate([x for x, _ in aligned])
    allin = np.concatenate([c for _, c in aligned])
    pc = o3d.t.geometry.PointCloud()
    pc.point['positions'] = o3d.core.Tensor(allxyz.astype(np.float32))
    pc.point['intensity'] = o3d.core.Tensor(allin.reshape(-1, 1).astype(np.float32))
    if a.out_voxel > 0:
        pc = pc.voxel_down_sample(a.out_voxel)
    o3d.t.io.write_point_cloud(out, pc, write_ascii=False)
    print(f"\n[icp] wrote {out}  ({pc.point['positions'].shape[0]:,} pts)")


if __name__ == '__main__':
    main()
