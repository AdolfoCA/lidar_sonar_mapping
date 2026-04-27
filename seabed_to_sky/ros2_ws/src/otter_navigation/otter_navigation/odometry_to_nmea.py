#!/usr/bin/env python3
"""
ROS2 Node: Odometry to NMEA Converter over TCP

Subscribes to /odometry topic and converts position/heading to NMEA GGA/RMC
sentences, then sends them via TCP to the autopilot.

Configuration: See config/otter.yaml

Usage:
  ros2 run otter_navigation odometry_to_nmea
"""

import math
import os
import socket
import threading
import time
import yaml
import logging

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from datetime import datetime


# Conversion factor: metres per second → knots
_MS_TO_KNOTS = 1.94384


class OdometryToNMEA(Node):
    """
    Converts odometry data to NMEA GGA/RMC sentences and sends via TCP.

    Safety behaviour
    ----------------
    NMEA output is suspended (and a warning logged) when any of the following
    conditions are detected:

    * No odometry message received for longer than ``odom_timeout`` seconds.
    * The incoming position or heading contains NaN or Inf.
    * The speed derived from ``twist.twist.linear`` exceeds ``max_speed_knots``.

    Output resumes automatically as soon as valid data arrives again.
    """

    def __init__(self):
        super().__init__('odometry_to_nmea')

        # Load configuration
        config_file = os.path.expanduser("~/nmea_config.yaml")
        self.config = self._load_config(config_file)

        self._setup_logging()
        self.logger = logging.getLogger(__name__)

        # --- Network / NMEA parameters ----------------------------------------
        origin = self.config['origin']
        network = self.config['network']
        nmea_cfg = self.config['nmea']

        self.origin_lat = origin['latitude']
        self.origin_lon = origin['longitude']
        self.autopilot_ip = network['autopilot_ip']
        self.autopilot_port = network['autopilot_port']
        self.publish_rate = nmea_cfg['publish_rate']
        self.gps_quality = nmea_cfg['gps_quality']
        self.num_satellites = nmea_cfg['num_satellites']
        self.hdop = nmea_cfg['hdop']

        # --- Validation parameters --------------------------------------------
        val = self.config.get('validation', {})
        self.odom_timeout = float(val.get('odom_timeout', 2.0))
        self.max_speed_knots = float(val.get('max_speed_knots', 2.0))

        # --- Odometry state (protected by lock) -------------------------------
        self.lock = threading.Lock()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.current_speed_knots = 0.0
        self.last_odom_time: float | None = None   # monotonic clock
        self.data_valid = False
        self._suspension_logged = False            # suppress repeated warnings

        # --- TCP socket -------------------------------------------------------
        self.sock = None
        self._connect_to_autopilot()               # non-fatal; retried each publish

        # --- ROS subscribers / timers -----------------------------------------
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry',
            self._odometry_callback,
            10,
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._publish_nmea)

        self.get_logger().info(
            f"Node initialised. Origin: ({self.origin_lat}°, {self.origin_lon}°) | "
            f"Autopilot: {self.autopilot_ip}:{self.autopilot_port} | "
            f"Rate: {self.publish_rate} Hz | "
            f"Timeout: {self.odom_timeout}s | "
            f"Max speed: {self.max_speed_knots} kn"
        )

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------

    def _load_config(self, config_file: str) -> dict:
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_file}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing config file: {e}")
            raise

    def _setup_logging(self):
        log_cfg = self.config.get('logging', {})
        log_level = getattr(logging, log_cfg.get('level', 'INFO'))
        handlers = [logging.StreamHandler()]
        if log_cfg.get('log_file'):
            handlers.append(logging.FileHandler(log_cfg['log_file']))
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=handlers,
        )

    # ------------------------------------------------------------------
    # TCP connection
    # ------------------------------------------------------------------

    def _connect_to_autopilot(self):
        """Try to open the TCP connection. Non-fatal — sets self.sock = None on failure."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((self.autopilot_ip, self.autopilot_port))
            sock.settimeout(None)
            self.sock = sock
            self.get_logger().info(
                f"Connected to autopilot at {self.autopilot_ip}:{self.autopilot_port}"
            )
        except socket.error as e:
            self.get_logger().warning(f"Cannot connect to autopilot: {e} — will retry")
            self.sock = None

    # ------------------------------------------------------------------
    # Odometry callback + validation
    # ------------------------------------------------------------------

    def _odometry_callback(self, msg: Odometry):
        """Validate and store the latest odometry message."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        heading = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # 1. Sanity: no NaN / Inf in position or heading
        if not all(math.isfinite(v) for v in (x, y, heading)):
            self._suspend("Odometry contains NaN or Inf values")
            return

        # 2. Speed check — derived from the velocity reported in the message
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        if not all(math.isfinite(v) for v in (vx, vy)):
            self._suspend("Odometry twist contains NaN or Inf values")
            return

        speed_knots = math.hypot(vx, vy) * _MS_TO_KNOTS
        if speed_knots > self.max_speed_knots:
            self._suspend(
                f"Speed {speed_knots:.2f} kn exceeds limit {self.max_speed_knots:.1f} kn"
            )
            return

        # All checks passed — update shared state
        with self.lock:
            self.current_x = x
            self.current_y = y
            self.current_heading = heading
            self.current_speed_knots = speed_knots
            self.last_odom_time = time.monotonic()
            was_suspended = not self.data_valid
            self.data_valid = True
            self._suspension_logged = False

        if was_suspended:
            self.get_logger().info("Odometry valid — NMEA output resumed")

    def _suspend(self, reason: str):
        """Mark data as invalid and log once per suspension event."""
        with self.lock:
            was_valid = self.data_valid
            self.data_valid = False
            already_logged = self._suspension_logged
            self._suspension_logged = True

        if was_valid or not already_logged:
            self.get_logger().warning(f"NMEA output suspended: {reason}")

    # ------------------------------------------------------------------
    # NMEA publishing
    # ------------------------------------------------------------------

    def _publish_nmea(self):
        """Timer callback: validate state then send GGA + RMC."""
        # Reconnect if socket dropped
        if self.sock is None:
            self._connect_to_autopilot()
            if self.sock is None:
                return

        with self.lock:
            last_time = self.last_odom_time
            valid = self.data_valid
            x = self.current_x
            y = self.current_y
            heading = self.current_heading
            speed_knots = self.current_speed_knots

        # Check for odometry timeout
        if last_time is None:
            self.get_logger().warning("Waiting for first odometry message…")
            return

        elapsed = time.monotonic() - last_time
        if elapsed > self.odom_timeout:
            self._suspend(f"No odometry for {elapsed:.1f}s (timeout: {self.odom_timeout}s)")
            return

        if not valid:
            return

        # Data is good — generate and send
        lat, lon = self._xy_to_latlon(x, y)
        gga = self._create_gga(lat, lon)
        rmc = self._create_rmc(lat, lon, heading, speed_knots)

        try:
            self.sock.sendall((gga + '\r\n').encode('utf-8'))
            self.sock.sendall((rmc + '\r\n').encode('utf-8'))
            self.get_logger().debug(f"Sent GGA: {gga}")
            self.get_logger().debug(f"Sent RMC: {rmc}")
        except socket.error as e:
            self.get_logger().error(f"TCP send error: {e}")
            self.sock = None

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def _xy_to_latlon(self, x: float, y: float):
        """
        Convert local ENU (x = North, y = East) metres to lat/lon.
        Small-angle approximation valid for areas < ~50 km radius.
        """
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(self.origin_lat))
        lat = self.origin_lat + x / m_per_deg_lat
        lon = self.origin_lon + y / m_per_deg_lon
        return lat, lon

    # ------------------------------------------------------------------
    # NMEA sentence builders
    # ------------------------------------------------------------------

    def _create_gga(self, lat: float, lon: float) -> str:
        """Build a GPGGA sentence."""
        now = datetime.utcnow()
        utc_time = now.strftime('%H%M%S.%f')[:-3]

        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = 'N' if lat >= 0 else 'S'

        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = 'E' if lon >= 0 else 'W'

        sentence = (
            f"GPGGA,{utc_time},"
            f"{lat_str},{lat_dir},{lon_str},{lon_dir},"
            f"{self.gps_quality},{self.num_satellites},{self.hdop},"
            f"0.0,M,0.0,M,,"
        )
        return self._add_checksum(sentence)

    def _create_rmc(self, lat: float, lon: float, heading: float, speed_knots: float) -> str:
        """Build a GPRMC sentence."""
        now = datetime.utcnow()
        utc_time = now.strftime('%H%M%S.%f')[:-3]
        date_str = now.strftime('%d%m%y')

        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = 'N' if lat >= 0 else 'S'

        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = 'E' if lon >= 0 else 'W'

        cog = math.degrees(heading) % 360

        sentence = (
            f"GPRMC,{utc_time},A,"
            f"{lat_str},{lat_dir},{lon_str},{lon_dir},"
            f"{speed_knots:.1f},{cog:06.1f},"
            f"{date_str},0.0,W"
        )
        return self._add_checksum(sentence)

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Extract yaw (radians) from a quaternion."""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _add_checksum(sentence: str) -> str:
        """Append NMEA XOR checksum: $<sentence>*<HH>"""
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"${sentence}*{checksum:02X}"

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        if self.sock:
            try:
                self.sock.close()
                self.get_logger().info("TCP connection closed")
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToNMEA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
