#!/usr/bin/env python3
"""Shared helpers for perf_tools: QoS profiles and config loading.

Kept dependency-light (only rclpy + pyyaml) so every capture node can import it.
"""

from __future__ import annotations

import os

import yaml
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


def sensor_qos(depth: int = 5) -> QoSProfile:
    """Best-effort sensor-data QoS (matches Ouster/PointCloud publishers)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=depth,
    )


def reliable_qos(depth: int = 10) -> QoSProfile:
    """Reliable QoS for low-rate topics (odometry, debug insert stamp)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=depth,
    )


def qos_for(kind: str, depth: int = 5) -> QoSProfile:
    """Resolve a per-topic 'qos' string from config to a QoSProfile."""
    if kind == 'reliable':
        return reliable_qos(max(depth, 10))
    return sensor_qos(depth)


def default_config_path() -> str:
    """config.yaml lives next to the package dir (src tree) or in share/."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, '..', 'config.yaml'),          # src tree
        os.path.join(here, 'config.yaml'),
    ]
    # installed share/ location
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            os.path.join(get_package_share_directory('perf_tools'), 'config.yaml'))
    except Exception:
        pass
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    # fall back to the first src-tree candidate (used in error messages)
    return os.path.abspath(candidates[0])


def load_config(path: str | None = None) -> dict:
    """Load and return the perf_tools config dict."""
    if not path:
        path = default_config_path()
    with open(path, 'r') as fh:
        return yaml.safe_load(fh)
