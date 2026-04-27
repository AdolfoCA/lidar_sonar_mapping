"""Demo orchestrator node.

Exposes two services callable from Foxglove's Service Call panel:
  /demo/start  (std_srvs/srv/Trigger) — launch full pipeline in sequence
  /demo/stop   (std_srvs/srv/Trigger) — kill all launched processes

Publishes current status to /demo/status (std_msgs/msg/String) at 1 Hz.
"""

import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


# Each step: (human label, ros2 launch command as list)
PIPELINE = [
    (
        "tf_static_publisher",
        ["ros2", "launch", "tf_static_publisher", "tf_static_launch.py"],
    ),
    (
        "sonar3d_reconstruction",
        ["ros2", "launch", "sonar3d_reconstruction", "acoustic3d_launch.py"],
    ),
    (
        "spark_fast_lio",
        ["ros2", "launch", "spark_fast_lio", "os2.launch.yaml"],
    ),
    (
        "sonar_map",
        ["ros2", "launch", "sonar_map", "sonar_mapping_no_semantic.launch.py"],
    ),
]

# Seconds to wait after each launch before starting the next
STEP_DELAY = 3.0


class DemoManager(Node):
    def __init__(self):
        super().__init__("demo_manager")

        self._processes: list[subprocess.Popen] = []
        self._status = "idle"
        self._lock = threading.Lock()

        self._start_srv = self.create_service(
            Trigger, "/demo/start", self._start_callback
        )
        self._stop_srv = self.create_service(
            Trigger, "/demo/stop", self._stop_callback
        )
        self._status_pub = self.create_publisher(String, "/demo/status", 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info("Demo manager ready — call /demo/start to begin")

    # ------------------------------------------------------------------
    def _start_callback(self, request, response):
        with self._lock:
            if self._processes:
                response.success = False
                response.message = "Pipeline already running — call /demo/stop first"
                return response

        self.get_logger().info("Starting demo pipeline...")
        threading.Thread(target=self._launch_sequence, daemon=True).start()
        response.success = True
        response.message = "Pipeline launch sequence started"
        return response

    def _stop_callback(self, request, response):
        killed = self._kill_all()
        response.success = True
        response.message = f"Stopped {killed} process(es)"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    def _launch_sequence(self):
        with self._lock:
            self._status = "launching..."

        for label, cmd in PIPELINE:
            self.get_logger().info(f"[DEMO] Starting: {label}")
            with self._lock:
                self._status = f"launching: {label}"
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                with self._lock:
                    self._processes.append(proc)
            except Exception as e:
                self.get_logger().error(f"[DEMO] Failed to launch {label}: {e}")
                with self._lock:
                    self._status = f"ERROR launching {label}"
                return

            time.sleep(STEP_DELAY)

        with self._lock:
            self._status = "running"
        self.get_logger().info("[DEMO] All components running")

    def _kill_all(self) -> int:
        with self._lock:
            procs = list(self._processes)
            self._processes.clear()
            self._status = "idle"

        for proc in procs:
            try:
                proc.terminate()
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception:
                pass

        return len(procs)

    def _publish_status(self):
        with self._lock:
            status = self._status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._kill_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
