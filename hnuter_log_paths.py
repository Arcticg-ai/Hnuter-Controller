#!/usr/bin/env python3
"""Shared log path helpers for Hnuter ROS 2 scripts."""

import os
import time
from pathlib import Path


def workspace_root() -> Path:
    return Path(__file__).resolve().parent


def log_root() -> Path:
    root = Path(os.environ.get('HNUTER_LOG_DIR', workspace_root() / 'hnuter_logs')).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    return root


def log_path(*parts: str) -> Path:
    path = log_root().joinpath(*parts)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def stamp() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


def configure_ros_log_dir() -> Path:
    ros_dir = log_path('ros', '.keep').parent
    os.environ.setdefault('ROS_LOG_DIR', str(ros_dir))
    return ros_dir


def diagnostic_csv_path(prefix: str) -> Path:
    return log_path('external_control', f'{prefix}_{int(time.time())}.csv')


def tuning_csv_path(prefix: str) -> Path:
    return log_path('tuning', f'{prefix}_{stamp()}.csv')
