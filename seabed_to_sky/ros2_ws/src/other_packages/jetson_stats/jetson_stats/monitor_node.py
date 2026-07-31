"""Jetson hardware monitor node.

Reads CPU and RAM directly from /proc — no external
dependencies required.

Published topics (all at ~2 Hz):
  /jetson/cpu_total      std_msgs/Float32   overall CPU usage [%]
  /jetson/cpu_per_core   std_msgs/Float32MultiArray  per-core usage [%]
  /jetson/ram_used_mb    std_msgs/Float32   RAM used [MB]
  /jetson/ram_percent    std_msgs/Float32   RAM used [%]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


# ---------------------------------------------------------------------------
# /proc readers
# ---------------------------------------------------------------------------

def _read_cpu_times():
    """Return list of (idle, total) per CPU line from /proc/stat."""
    results = []
    with open("/proc/stat") as f:
        for line in f:
            if not line.startswith("cpu"):
                break
            parts = line.split()
            if parts[0] == "cpu":
                continue  # skip the aggregate line — we compute it ourselves
            values = list(map(int, parts[1:]))
            # idle = idle + iowait (indices 3 and 4)
            idle  = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
            results.append((idle, total))
    return results


def _read_mem_kb():
    """Return (total_kb, available_kb) from /proc/meminfo."""
    total = available = None
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                total = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                available = int(line.split()[1])
            if total is not None and available is not None:
                break
    return total, available


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class JetsonMonitor(Node):
    def __init__(self):
        super().__init__("jetson_stats")

        rate_hz = self.declare_parameter("rate_hz", 2.0).value

        qos = rclpy.qos.QoSProfile(depth=10)
        self._pub_cpu_total    = self.create_publisher(Float32,           "/jetson/cpu_total",    qos)
        self._pub_cpu_cores    = self.create_publisher(Float32MultiArray, "/jetson/cpu_per_core", qos)
        self._pub_ram_mb       = self.create_publisher(Float32,           "/jetson/ram_used_mb",  qos)
        self._pub_ram_pct      = self.create_publisher(Float32,           "/jetson/ram_percent",  qos)

        # Seed the first CPU sample so delta is meaningful on first publish
        self._prev_cpu = _read_cpu_times()

        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(f"Jetson monitor running at {rate_hz} Hz")

    def _publish(self):
        now = self.get_clock().now().to_msg()

        # ── CPU ──────────────────────────────────────────────────────────────
        curr_cpu = _read_cpu_times()
        core_pcts = []
        for (prev_idle, prev_total), (curr_idle, curr_total) in zip(self._prev_cpu, curr_cpu):
            d_total = curr_total - prev_total
            d_idle  = curr_idle  - prev_idle
            pct = 100.0 * (1.0 - d_idle / d_total) if d_total > 0 else 0.0
            core_pcts.append(max(0.0, min(100.0, pct)))
        self._prev_cpu = curr_cpu

        total_pct = sum(core_pcts) / len(core_pcts) if core_pcts else 0.0

        msg_total      = Float32(); msg_total.data      = float(total_pct)
        msg_cores      = Float32MultiArray(); msg_cores.data = [float(p) for p in core_pcts]
        self._pub_cpu_total.publish(msg_total)
        self._pub_cpu_cores.publish(msg_cores)

        # ── RAM ──────────────────────────────────────────────────────────────
        total_kb, avail_kb = _read_mem_kb()
        if total_kb and avail_kb:
            used_kb  = total_kb - avail_kb
            used_mb  = used_kb  / 1024.0
            ram_pct  = 100.0 * used_kb / total_kb

            msg_mb  = Float32(); msg_mb.data  = float(used_mb)
            msg_pct = Float32(); msg_pct.data = float(ram_pct)
            self._pub_ram_mb.publish(msg_mb)
            self._pub_ram_pct.publish(msg_pct)



def main(args=None):
    rclpy.init(args=args)
    node = JetsonMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
