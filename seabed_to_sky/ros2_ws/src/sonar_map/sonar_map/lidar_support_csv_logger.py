#!/usr/bin/env python3
"""
lidar_support_csv_logger — debug logger for the LiDAR support channels (d, k).
===============================================================================

A PURE OBSERVER. Does not feed the pipeline. It exists to let us debug, offline,
the raw LiDAR-support measurements that drive the STRUCTURE/OBJECT likelihoods
in voxel_mapper (paper eqs 28–31).

For every sonar return in each /classified_returns batch it writes ONE CSV row
carrying:
  - the scan timestamp (so rows group by scan),
  - the return position (x,y,z, odom) + intensity + slant_range,
  - the seabed-intensity context (mu_i, sigma_i),
  - the PUBLISHED LiDAR support (lidar_dist d_pub, lidar_count k_pub) — what the
    classifier shipped and voxel_mapper consumed, and
  - the RE-DERIVED nearest-LiDAR distance `d_calc` and neighbour `k_calc`,
    rebuilt here from /cloud_registered with the SAME XY KD-tree + eps_L the
    classifier uses, so any mismatch vs the published values is visible.

Params (default to the LIVE return_classifier values so the recomputation
matches production — pass --ros-args -p ... to override):
  input_topic   classified_returns
  lidar_topic   /cloud_registered
  eps_L         0.15   [m]  support-count radius (MUST match classifier)
  eps_Lz        0.0    [m]  z half-height of the count cylinder; ≤0 = pure XY
                            (MUST match classifier)
  csv_path      ~/lidar_support_debug.csv
  enable_csv    False  master on/off switch
"""

import csv
import os

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy.lib.recfunctions as rfn
from scipy.spatial import cKDTree

from sonar_map_msgs.msg import ClassifiedReturns


def _read_xyz(msg: PointCloud2) -> np.ndarray:
    raw = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        if raw.dtype.names is not None:
            xyz = rfn.structured_to_unstructured(raw, dtype=np.float64)
        else:
            xyz = raw.reshape(-1, 3).astype(np.float64)
    else:
        rows = list(raw)
        if not rows:
            return np.empty((0, 3), dtype=np.float64)
        xyz = np.array(rows, dtype=np.float64).reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)] if xyz.size else xyz


class LidarSupportCsvLogger(Node):

    def __init__(self):
        super().__init__('lidar_support_csv_logger')

        self.declare_parameter('input_topic',  'classified_returns')
        self.declare_parameter('lidar_topic',  '/cloud_registered')
        self.declare_parameter('eps_L',        0.15)
        self.declare_parameter('eps_Lz',       0.0)
        self.declare_parameter('csv_path',
                               os.path.expanduser('~/lidar_support_debug.csv'))
        # Master on/off switch: when False this node opens no file, subscribes
        # to nothing and does no work — pure no-op. Flip True in the config to
        # capture the debug CSV.
        self.declare_parameter('enable_csv', False)

        def gp(n):
            return self.get_parameter(n).value

        self._enable_csv  = bool(gp('enable_csv'))
        self._eps_l       = max(float(gp('eps_L')), 1e-6)
        self._eps_lz      = float(gp('eps_Lz'))   # ≤0 ⇒ z band off (pure XY)
        self._csv_path    = os.path.expanduser(str(gp('csv_path')))

        self._lidar_tree   = None       # cKDTree over LiDAR XY (horizontal)
        self._lidar_z      = None       # (M,) float64 z of each LiDAR pt (for the band)
        self._lidar_stamp  = None       # (sec, nsec) of the cached LiDAR cloud
        self._lidar_npts   = 0
        self._rows_written = 0
        self._fh  = None
        self._csv = None

        # CSV disabled: stay idle. No file, no subscriptions, no compute.
        if not self._enable_csv:
            self.get_logger().info(
                'lidar_support_csv_logger: enable_csv=False — CSV logging '
                'disabled, node idle. Set enable_csv:=true to capture.')
            return

        self._fh = open(self._csv_path, 'w', newline='')
        self._csv = csv.writer(self._fh)
        self._csv.writerow([
            'scan_sec', 'scan_nsec', 'idx',
            'x', 'y', 'z', 'intensity', 'mu_i', 'sigma_i', 'slant_range',
            'd_pub',              # lidar_dist shipped by the classifier [m]
            'k_pub',              # lidar_count shipped by the classifier
            'd_calc',             # re-derived nearest-LiDAR distance [m]
            'k_calc',             # re-derived # LiDAR pts within eps_L of q*
            'lidar_stamp_sec', 'lidar_stamp_nsec', 'lidar_npts',
        ])
        self._fh.flush()

        self.sub_lidar_ = self.create_subscription(
            PointCloud2, str(gp('lidar_topic')), self._lidar_cb, 1)
        self.sub_returns_ = self.create_subscription(
            ClassifiedReturns, str(gp('input_topic')), self._returns_cb, 1)

        self.get_logger().info(
            f'lidar_support_csv_logger -> {self._csv_path} | '
            f'eps_L={self._eps_l}m (XY)'
            + (f' ∩ |Δz|≤{self._eps_lz}m' if self._eps_lz > 0.0
               else ' [z band off]'))

    def _lidar_cb(self, msg: PointCloud2):
        xyz = _read_xyz(msg)
        if xyz.shape[0] == 0:
            self._lidar_tree = None
            self._lidar_z    = None
            self._lidar_npts = 0
            return
        self._lidar_tree  = cKDTree(xyz[:, :2])   # XY-only, matches classifier
        self._lidar_z     = xyz[:, 2].astype(np.float64)  # for the optional z band
        self._lidar_npts  = xyz.shape[0]
        self._lidar_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _returns_cb(self, msg: ClassifiedReturns):
        x = np.asarray(msg.x, dtype=np.float64)
        n = x.shape[0]
        if n == 0:
            return
        y = np.asarray(msg.y, dtype=np.float64)
        z = np.asarray(msg.z, dtype=np.float64)
        inten = np.asarray(msg.intensity,   dtype=np.float64)
        slant = np.asarray(msg.slant_range, dtype=np.float64)
        d_pub = np.asarray(msg.lidar_dist,  dtype=np.float64)
        k_pub = np.asarray(msg.lidar_count, dtype=np.float64)
        mu_i    = float(msg.mu_i)
        sigma_i = float(msg.sigma_i)

        sonar_xyz = np.column_stack((x, y, z))

        # Re-derive d (nearest-LiDAR distance) and k, same query as the
        # classifier: nearest LiDAR pt q* in the XY plane (z ignored for q*/d),
        # then # LiDAR pts within eps_L of q* in XY — intersected with the z band
        # |c_z − q*_z| ≤ eps_Lz when eps_Lz > 0 (the cylinder count).
        if self._lidar_tree is not None:
            sonar_xy = sonar_xyz[:, :2]
            d_calc, idx = self._lidar_tree.query(sonar_xy, k=1)
            d_calc = np.asarray(d_calc, dtype=np.float64).reshape(n)
            idx = np.asarray(idx, dtype=np.int64).reshape(n)
            qstar = np.asarray(self._lidar_tree.data, dtype=np.float64)[idx]
            if self._eps_lz <= 0.0 or self._lidar_z is None:
                k_calc = np.asarray(self._lidar_tree.query_ball_point(
                    qstar, r=self._eps_l, return_length=True), dtype=np.float64)
            else:
                qstar_z = self._lidar_z[idx]
                ball    = self._lidar_tree.query_ball_point(qstar, r=self._eps_l)
                lz      = self._lidar_z
                k_calc  = np.empty(n, dtype=np.float64)
                for i in range(n):
                    members = ball[i]
                    if members:
                        dz = np.abs(lz[members] - qstar_z[i])
                        k_calc[i] = float(np.count_nonzero(dz <= self._eps_lz))
                    else:
                        k_calc[i] = 0.0
            lstamp = self._lidar_stamp or (-1, 0)
        else:
            d_calc = np.full(n, np.inf)
            k_calc = np.zeros(n)
            lstamp = (-1, 0)

        sec  = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec

        for i in range(n):
            self._csv.writerow([
                sec, nsec, i,
                f'{x[i]:.4f}', f'{y[i]:.4f}', f'{z[i]:.4f}',
                f'{inten[i]:.2f}', f'{mu_i:.2f}', f'{sigma_i:.2f}',
                f'{slant[i]:.3f}',
                f'{d_pub[i]:.4f}', int(k_pub[i]),
                f'{d_calc[i]:.4f}', int(k_calc[i]),
                lstamp[0], lstamp[1], self._lidar_npts,
            ])
            self._rows_written += 1

        self._fh.flush()
        if self._rows_written and self._rows_written % 2000 < n:
            self.get_logger().info(
                f'{self._rows_written} rows written to {self._csv_path}')

    def destroy_node(self):
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSupportCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Stopping. {node._rows_written} rows -> {node._csv_path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
