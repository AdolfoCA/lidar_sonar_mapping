#!/usr/bin/env python3
"""
mission_supervisor — long-mission map health via ODOMETRY-LATENCY watchdog.
===============================================================================

Instead of watching RAM, this watches the odometry COMPUTE latency published by
spark_fast_lio on `odometry_latency_ms` (per-scan EKF-update time, which grows as
the LiDAR map grows). At the start the system is healthy, so we learn a BASELINE
latency; when the (smoothed) latency exceeds baseline by `latency_exceed_percent`
for a few checks in a row, we fire ONE prune:

  1. SONAR : call /sonar_map/save_and_reset  (save the sonar map, then clear it)
  2. LIDAR : call /prune_kdtree              (save the FULL lidar map, then delete
             the furthest `lidar_prune_fraction` of points by distance from the
             vessel — the spark_fast_lio side floors the keep radius at cube_len/2
             so the FAST-LIO registration map, and thus the odometry, is safe)

One 'prune' row is written to mission/events.csv per trigger (with the vessel
position and the latency/baseline), so every plot shows a single prune line.

The TWO tuning knobs live here (mission_supervisor.yaml):
  * latency_exceed_percent — how far over baseline before pruning
  * lidar_prune_fraction   — fraction of furthest lidar points to delete; this is
                             pushed to the spark_fast_lio node's `prune.fraction`.
"""

import csv
import time
from collections import deque
from pathlib import Path
from statistics import median

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType


def _as_bool(v):
    """Coerce a param value (bool / int / "true"/"false" string) to bool."""
    if isinstance(v, str):
        return v.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(v)


class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        p = self.declare_parameter

        self.mission_dir = Path(p('mission_dir', '/home/rosdev/ros2_ws/mission').value)
        self.enable_actions = _as_bool(p('enable_actions', True).value)
        self.eval_period_s = float(p('eval_period_s', 1.0).value)
        self.plot_on_exit = _as_bool(p('plot_on_exit', True).value)

        # ---- ODOMETRY-LATENCY trigger ----
        self.latency_topic = str(p('odom_latency_topic', '/odometry_latency_ms').value)
        self.latency_exceed_percent = float(p('latency_exceed_percent', 50.0).value)  # KNOB 1
        self.baseline_window_s = float(p('baseline_window_s', 30.0).value)  # healthy-start window
        self.baseline_min_samples = int(p('baseline_min_samples', 30).value)
        self.latency_smooth_n = int(p('latency_smooth_n', 20).value)        # median smoothing
        self.latency_consecutive = int(p('latency_consecutive', 3).value)   # debounce
        self.latency_cooldown_s = float(p('latency_cooldown_s', 30.0).value)

        # ---- LiDAR prune fraction (KNOB 2) pushed to the spark_fast_lio node ----
        self.lidar_prune_fraction = float(p('lidar_prune_fraction', 0.5).value)
        self.lidar_node_name = str(p('lidar_node_name', 'lio_mapping').value)

        # ---- services fired together on the trigger ----
        self.lidar_service = str(p('lidar_prune_service', '/prune_kdtree').value)
        self.sonar_service = str(p('sonar_reset_service', '/sonar_map/save_and_reset').value)
        self.odom_topic = str(p('odom_topic', '/odometry').value)

        # ---- state ----
        self._lat = deque(maxlen=max(self.latency_smooth_n * 5, 100))  # recent latency [ms]
        self._baseline_vals = []
        self._t_first = None                # monotonic of the first latency sample
        self._baseline = None               # learned baseline latency [ms]
        self._over = 0
        self._last_prune_mono = -1e9
        self._last_pose = None              # (x, y, z)
        self._frac_pushed = False
        self._frac_inflight = False

        # mission/ layout + events.csv.
        for sub in ('perf_data', 'sonar', 'lidar', 'plots'):
            (self.mission_dir / sub).mkdir(parents=True, exist_ok=True)
        self.events_path = self.mission_dir / 'events.csv'
        if not self.events_path.exists():
            with open(self.events_path, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['wall_ns', 'monotonic_ns', 'action', 'chain', 'value', 'note'])

        self.lat_pub = self.create_publisher(Float32, '/mission/odom_latency_ms', 10)
        self.lidar_cli = self.create_client(Trigger, self.lidar_service)
        self.sonar_cli = self.create_client(Trigger, self.sonar_service)
        self.create_subscription(Float64, self.latency_topic, self._lat_cb, 50)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        # Push the LiDAR prune fraction to the spark_fast_lio node (retry until up).
        self._frac_cli = self.create_client(
            SetParameters, f'/{self.lidar_node_name}/set_parameters')
        self.create_timer(3.0, self._push_frac)

        self.create_timer(self.eval_period_s, self._evaluate)
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"mission_supervisor: actions={'ON' if self.enable_actions else 'OFF (monitor only)'} | "
            f"watch {self.latency_topic} | learn baseline over {self.baseline_window_s:g}s "
            f"-> fire when latency >= baseline x (1 + {self.latency_exceed_percent:g}%) "
            f"for {self.latency_consecutive}x{self.eval_period_s:g}s | "
            f"prune BOTH ({self.lidar_service} frac={self.lidar_prune_fraction:g} + "
            f"{self.sonar_service}) | cooldown={self.latency_cooldown_s:g}s | events -> {self.events_path}")

    # ----- push prune fraction to spark_fast_lio -----
    def _push_frac(self):
        if self._frac_pushed or self._frac_inflight:
            return
        if not self._frac_cli.service_is_ready():
            return
        req = SetParameters.Request()
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                            double_value=float(self.lidar_prune_fraction))
        req.parameters = [ParamMsg(name='prune.fraction', value=pv)]
        self._frac_inflight = True
        self._frac_cli.call_async(req).add_done_callback(self._on_frac_pushed)

    def _on_frac_pushed(self, fut):
        self._frac_inflight = False
        try:
            resp = fut.result()
            ok = bool(resp.results) and all(r.successful for r in resp.results)
            if ok:
                self._frac_pushed = True
                self.get_logger().info(
                    f"pushed prune.fraction={self.lidar_prune_fraction:g} to {self.lidar_node_name}")
            else:
                self.get_logger().warn("prune.fraction push rejected; will retry")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"prune.fraction push failed: {exc!r}; will retry")

    # ----- inputs -----
    def _lat_cb(self, msg):
        v = float(msg.data)
        now = time.monotonic()
        self._lat.append(v)
        if self._baseline is None:
            if self._t_first is None:
                self._t_first = now
            if (now - self._t_first) <= self.baseline_window_s:
                self._baseline_vals.append(v)

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        self._last_pose = (pos.x, pos.y, pos.z)

    def _pose_str(self):
        if self._last_pose is None:
            return 'unknown (no odom yet)'
        return 'x=%.2f y=%.2f z=%.2f' % self._last_pose

    # ----- live parameter updates -----
    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            name, val = prm.name, prm.value
            if name == 'latency_exceed_percent':
                self.latency_exceed_percent = float(val)
            elif name == 'latency_cooldown_s':
                self.latency_cooldown_s = float(val)
            elif name == 'latency_consecutive':
                self.latency_consecutive = int(val)
            elif name == 'enable_actions':
                self.enable_actions = _as_bool(val)
        self.get_logger().info(
            f"params updated: exceed={self.latency_exceed_percent:g}% "
            f"x{self.latency_consecutive} cooldown={self.latency_cooldown_s:g}s "
            f"actions={'ON' if self.enable_actions else 'OFF'}")
        return SetParametersResult(successful=True)

    # ----- evaluation + action -----
    def _evaluate(self):
        if not self._lat:
            return
        smoothed = median(list(self._lat)[-self.latency_smooth_n:])
        self.lat_pub.publish(Float32(data=float(smoothed)))

        # Learn the baseline once the healthy-start window has elapsed.
        if self._baseline is None:
            if self._t_first is None:
                return
            if (time.monotonic() - self._t_first) <= self.baseline_window_s:
                return
            if len(self._baseline_vals) < self.baseline_min_samples:
                return  # keep collecting until we have enough samples
            self._baseline = float(median(self._baseline_vals))
            self.get_logger().info(
                f"baseline odometry latency = {self._baseline:.2f} ms "
                f"(from {len(self._baseline_vals)} samples) -> "
                f"prune threshold = {self._baseline * (1 + self.latency_exceed_percent / 100.0):.2f} ms")
            return

        threshold = self._baseline * (1.0 + self.latency_exceed_percent / 100.0)
        self._over = self._over + 1 if smoothed >= threshold else 0
        if self._over < self.latency_consecutive:
            return
        mono = time.monotonic()
        if (mono - self._last_prune_mono) < self.latency_cooldown_s:
            return
        self._last_prune_mono = mono
        self._over = 0
        self._fire(smoothed)

    def _fire(self, smoothed):
        """Fire SONAR save_and_reset + LIDAR percentile prune; one 'prune' row."""
        pos = self._pose_str()
        ctx = (f"latency {smoothed:.1f} ms >= {self._baseline * (1 + self.latency_exceed_percent / 100.0):.1f} ms "
               f"(baseline {self._baseline:.1f} +{self.latency_exceed_percent:g}%)")
        if not self.enable_actions:
            self._log_event('prune_SKIPPED', 'latency', smoothed, f'actions disabled; {ctx}; pos={pos}')
            self.get_logger().info(f"{ctx} but actions OFF | POSITION {pos}")
            return
        fired, missing = [], []
        for cli, tag in ((self.lidar_cli, 'lidar'), (self.sonar_cli, 'sonar')):
            if cli.service_is_ready():
                cli.call_async(Trigger.Request()).add_done_callback(
                    lambda f, t=tag: self._on_srv_done(f, t))
                fired.append(tag)
            else:
                missing.append(f'{tag}:no_srv')
        self.get_logger().warn(
            f"{ctx} -> prune+save [{'+'.join(fired) if fired else 'none'}]"
            f"{' ' + ' '.join(missing) if missing else ''} | POSITION {pos}")
        note = f"{ctx}; fired={'+'.join(fired) or 'none'}; pos={pos}"
        if missing:
            note += '; ' + ' '.join(missing)
        self._log_event('prune', 'latency', smoothed, note)

    def _on_srv_done(self, future, tag):
        try:
            resp = future.result()
            self.get_logger().info(f"[{tag}] {'ok' if resp.success else 'FAIL'}: {resp.message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[{tag}] service call failed: {exc!r}")

    def _log_event(self, action, chain, value, note):
        with open(self.events_path, 'a', newline='') as f:
            csv.writer(f).writerow(
                [time.time_ns(), time.monotonic_ns(), action, chain, f'{value:.1f}', note])

    # ----- end-of-mission plots (best-effort, on shutdown) -----
    def generate_plots_on_exit(self):
        """Render mission/plots/*.png from the NEWEST perf_data/<run_id> + events.csv."""
        if not self.plot_on_exit:
            return
        try:
            import importlib.util
            import subprocess
            perf_root = self.mission_dir / 'perf_data'
            runs = sorted((d for d in perf_root.glob('*') if d.is_dir()),
                          key=lambda d: d.stat().st_mtime)
            if not runs:
                self.get_logger().warn('plot_on_exit: no perf_data/<run_id> found; skipping plots')
                return
            run_dir = runs[-1]
            spec = importlib.util.find_spec('perf_tools.plot_perf_mission')
            if spec is None or not spec.origin:
                self.get_logger().warn('plot_on_exit: plot_perf_mission not importable; skipping')
                return
            cmd = ['python3', spec.origin, '--run-dir', str(run_dir),
                   '--mission-dir', str(self.mission_dir)]
            self.get_logger().info(f'plot_on_exit: rendering mission plots from {run_dir} ...')
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
            if r.returncode == 0:
                self.get_logger().info(f'plot_on_exit: plots -> {self.mission_dir / "plots"}')
            else:
                self.get_logger().warn(f'plot_on_exit: plotter exited {r.returncode}: {r.stderr[-300:]}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'plot_on_exit: skipped ({exc!r})')


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_plots_on_exit()      # best-effort end-of-mission plots
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
