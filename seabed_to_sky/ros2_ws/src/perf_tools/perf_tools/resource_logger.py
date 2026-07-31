#!/usr/bin/env python3
"""resource_logger.py — 1 Hz system + per-process resource logger.

Writes resources.csv with, per second:
    wall_ns, monotonic_ns,
    cpu0..cpuN (per-core %), cpu_total,
    ram_used_mb, ram_total_mb,
    gpu_pct, power_mw, temp_<zone>...,
    proc_<label>_cpu, proc_<label>_rss_mb   (for each configured component)

Telemetry backend priority:  jtop (jetson-stats)  ->  tegrastats  ->  psutil-only.
On a PC (no Jetson), GPU/power/SoC-temp fields are written empty and a warning is
printed once; per-core CPU + RAM + per-process always work via psutil.

Standalone (no ROS needed):
    python3 resource_logger.py --run-id 20260615_120000 [--config config.yaml]
                               [--interval 1.0] [--out-root ../../perf_data]
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time

import psutil

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


# --------------------------------------------------------------------------- #
# config loading (kept independent of rclpy so this runs on a bare laptop too) #
# --------------------------------------------------------------------------- #
def _default_config_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, '..', 'config.yaml'))


def load_config(path: str | None) -> dict:
    if not path:
        path = _default_config_path()
    if yaml is None or not os.path.isfile(path):
        return {}
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


# --------------------------------------------------------------------------- #
# Jetson telemetry backends                                                    #
# --------------------------------------------------------------------------- #
class JtopBackend:
    """jetson-stats (jtop) python API. Best source on a Jetson."""

    def __init__(self):
        from jtop import jtop  # raises if not installed
        self._jtop = jtop()
        self._jtop.start()

    def sample(self) -> dict:
        j = self._jtop
        out = {'gpu_pct': '', 'power_mw': '', 'temps': {}}
        try:
            gpu = j.gpu
            # jtop schema varies by version; try common shapes
            if isinstance(gpu, dict):
                first = next(iter(gpu.values()))
                if isinstance(first, dict):
                    out['gpu_pct'] = first.get('status', {}).get('load', first.get('load', ''))
        except Exception:
            pass
        try:
            power = j.power
            if isinstance(power, dict):
                tot = power.get('tot', {})
                out['power_mw'] = tot.get('power', '') if isinstance(tot, dict) else ''
        except Exception:
            pass
        try:
            for name, val in (j.temperature or {}).items():
                t = val.get('temp', '') if isinstance(val, dict) else val
                out['temps'][f'temp_{name}'] = t
        except Exception:
            pass
        return out

    def close(self):
        try:
            self._jtop.close()
        except Exception:
            pass


class TegrastatsBackend:
    """Parse the streaming output of `tegrastats`."""

    def __init__(self, interval_ms: int = 1000):
        self._proc = subprocess.Popen(
            ['tegrastats', '--interval', str(interval_ms)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self._lock = threading.Lock()
        self._last_line = ''
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        for line in self._proc.stdout:
            with self._lock:
                self._last_line = line.strip()

    def sample(self) -> dict:
        import re
        with self._lock:
            line = self._last_line
        out = {'gpu_pct': '', 'power_mw': '', 'temps': {}}
        if not line:
            return out
        m = re.search(r'GR3D_FREQ (\d+)%', line)
        if m:
            out['gpu_pct'] = int(m.group(1))
        # POM / VDD power rails in mW, e.g. "VDD_IN 5071mW/5071mW"
        powers = re.findall(r'(\w+) (\d+)mW/\d+mW', line)
        for rail, mw in powers:
            if rail in ('VDD_IN', 'POM_5V_IN'):
                out['power_mw'] = int(mw)
        if out['power_mw'] == '' and powers:
            out['power_mw'] = sum(int(mw) for _, mw in powers)
        for name, val in re.findall(r'(\w+)@([\d.]+)C', line):
            out['temps'][f'temp_{name}'] = float(val)
        return out

    def close(self):
        try:
            self._proc.terminate()
        except Exception:
            pass


class PsutilTempBackend:
    """psutil-only fallback: no GPU/power, temps if the platform exposes them."""

    def close(self):
        pass

    def sample(self) -> dict:
        out = {'gpu_pct': '', 'power_mw': '', 'temps': {}}
        try:
            temps = psutil.sensors_temperatures()
            for chip, entries in (temps or {}).items():
                for e in entries:
                    label = e.label or chip
                    out['temps'][f'temp_{label}'.replace(' ', '_')] = e.current
        except Exception:
            pass
        return out


def make_backend(interval_ms: int):
    """Return (backend, name) using the highest-priority available source."""
    try:
        return JtopBackend(), 'jtop'
    except Exception:
        pass
    # tegrastats only useful on Jetson; check it exists
    try:
        if subprocess.run(['which', 'tegrastats'],
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            return TegrastatsBackend(interval_ms), 'tegrastats'
    except Exception:
        pass
    return PsutilTempBackend(), 'psutil'


# --------------------------------------------------------------------------- #
# Per-process attribution                                                      #
# --------------------------------------------------------------------------- #
class ProcTracker:
    """Find and follow the processes matching the configured components."""

    def __init__(self, specs: list[dict]):
        # specs: [{label, match}]
        self.specs = specs
        self.labels = [s['label'] for s in specs]
        self._procs: dict[str, list[psutil.Process]] = {s['label']: [] for s in specs}
        self.rescan()

    def rescan(self):
        for label in self._procs:
            self._procs[label] = []
        for p in psutil.process_iter(['name', 'cmdline']):
            try:
                name = p.info.get('name') or ''
                cmd = ' '.join(p.info.get('cmdline') or [])
                hay = name + ' ' + cmd
                for s in self.specs:
                    if s['match'] in hay:
                        self._procs[s['label']].append(p)
                        # prime cpu_percent so the next call is meaningful
                        try:
                            p.cpu_percent(None)
                        except Exception:
                            pass
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

    def sample(self) -> dict:
        out = {}
        any_missing = False
        for label, procs in self._procs.items():
            cpu = 0.0
            rss = 0.0
            alive = []
            for p in procs:
                try:
                    cpu += p.cpu_percent(None)
                    rss += p.memory_info().rss
                    alive.append(p)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    any_missing = True
            self._procs[label] = alive
            if not alive:
                out[f'proc_{label}_cpu'] = ''
                out[f'proc_{label}_rss_mb'] = ''
                any_missing = True
            else:
                out[f'proc_{label}_cpu'] = round(cpu, 2)
                out[f'proc_{label}_rss_mb'] = round(rss / 1e6, 2)
        return out, any_missing


def _resolve_run_dir(out_root, run_id) -> str:
    if not out_root:
        here = os.path.dirname(os.path.abspath(__file__))
        out_root = os.path.abspath(os.path.join(here, '..', '..', '..', 'perf_data'))
    return os.path.join(out_root, run_id)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='1 Hz system + per-process resource logger (jtop/tegrastats/psutil).')
    parser.add_argument('--run-id', default=None,
                        help='Run id (folder). Defaults to PERF_RUN_ID env or timestamp.')
    parser.add_argument('--config', default=None, help='Path to config.yaml')
    parser.add_argument('--interval', type=float, default=1.0, help='Sample period (s)')
    parser.add_argument('--out-root', default=None, help='Root dir for perf_data')
    if argv is None:
        # drop ROS-injected args (after --ros-args) so this runs cleanly under
        # `ros2 launch`/`ros2 run` as well as standalone.
        argv = sys.argv[1:]
        if '--ros-args' in argv:
            argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)

    run_id = args.run_id or os.environ.get('PERF_RUN_ID') or time.strftime('%Y%m%d_%H%M%S')
    cfg = load_config(args.config)
    specs = cfg.get('process_names', [])
    run_dir = _resolve_run_dir(args.out_root, run_id)
    os.makedirs(run_dir, exist_ok=True)
    csv_path = os.path.join(run_dir, 'resources.csv')

    backend, backend_name = make_backend(int(args.interval * 1000))
    if backend_name == 'psutil':
        print('[resource_logger] WARNING: no jtop/tegrastats — GPU/power unavailable '
              '(expected on a PC). Logging per-core CPU, RAM and per-process only.',
              file=sys.stderr)
    else:
        print(f'[resource_logger] telemetry backend: {backend_name}', file=sys.stderr)

    tracker = ProcTracker(specs)

    n_cores = psutil.cpu_count(logical=True) or 1
    # prime psutil cpu_percent (first call returns 0.0)
    psutil.cpu_percent(percpu=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *a: stop.set())
    signal.signal(signal.SIGTERM, lambda *a: stop.set())

    # discover the temp field names from one warm-up sample so the header is stable
    warm = backend.sample()
    temp_fields = sorted(warm.get('temps', {}).keys())

    header = (['wall_ns', 'monotonic_ns']
              + [f'cpu{i}' for i in range(n_cores)] + ['cpu_total']
              + ['ram_used_mb', 'ram_total_mb', 'gpu_pct', 'power_mw']
              + temp_fields
              + [c for label in tracker.labels
                 for c in (f'proc_{label}_cpu', f'proc_{label}_rss_mb')])

    rescan_every = max(1, int(10 / args.interval))  # re-scan processes ~every 10 s
    tick = 0
    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        print(f'[resource_logger] writing {csv_path} every {args.interval}s', file=sys.stderr)
        while not stop.is_set():
            t0 = time.monotonic()
            percore = psutil.cpu_percent(percpu=True)
            cpu_total = round(sum(percore) / len(percore), 2) if percore else ''
            vm = psutil.virtual_memory()
            sysd = backend.sample()
            if tick % rescan_every == 0:
                tracker.rescan()
            proc_d, _ = tracker.sample()

            row = [time.time_ns(), time.monotonic_ns()]
            row += [round(c, 2) for c in percore]
            # pad if core count changed
            if len(percore) < n_cores:
                row += [''] * (n_cores - len(percore))
            row += [cpu_total,
                    round(vm.used / 1e6, 2), round(vm.total / 1e6, 2),
                    sysd.get('gpu_pct', ''), sysd.get('power_mw', '')]
            row += [sysd.get('temps', {}).get(f, '') for f in temp_fields]
            row += [proc_d.get(c, '') for label in tracker.labels
                    for c in (f'proc_{label}_cpu', f'proc_{label}_rss_mb')]
            writer.writerow(row)
            fh.flush()

            tick += 1
            # sleep the remainder of the interval, interruptibly
            elapsed = time.monotonic() - t0
            stop.wait(max(0.0, args.interval - elapsed))

    backend.close()
    print('[resource_logger] stopped.', file=sys.stderr)


if __name__ == '__main__':
    main()
