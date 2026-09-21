"""Gazebo joint-angle feedback for the SITL-only closed-loop experiment."""

from __future__ import annotations

import math
import sys
import threading
import time

import numpy as np


def _quat(pose) -> np.ndarray:
    q = pose.orientation
    value = np.array([q.w, q.x, q.y, q.z], dtype=float)
    return value / max(float(np.linalg.norm(value)), 1e-12)


def _multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _inverse(q: np.ndarray) -> np.ndarray:
    return q * np.array([1.0, -1.0, -1.0, -1.0])


class JointAngleFeedback:
    """Measure four tilt angles from physical link poses, never command echoes.

    The four joints use local -Z axes in the Hnuter SDF, transformed to +Y
    in their parent links by the joint poses. Relative zero orientations are
    calibrated only while disarmed. This observer is SITL-only.
    """

    LINKS = (("base_link", "l2"), ("l2", "l1"),
             ("base_link", "r2"), ("r2", "r1"))

    def __init__(self, topic: str = "/world/default/dynamic_pose/info") -> None:
        if "/usr/lib/python3/dist-packages" not in sys.path:
            sys.path.append("/usr/lib/python3/dist-packages")
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self._lock = threading.Lock()
        self._zero: list[np.ndarray] | None = None
        self._angles: np.ndarray | None = None
        self._received_at = 0.0
        self._calibration_allowed = True
        self._node = Node()
        self._node.subscribe(Pose_V, topic, self._on_pose)

    @staticmethod
    def angles_from_poses(poses: dict, zero: list[np.ndarray]) -> np.ndarray:
        angles = []
        for (parent, child), reference in zip(JointAngleFeedback.LINKS, zero):
            relative = _multiply(_inverse(_quat(poses[parent])), _quat(poses[child]))
            delta = _multiply(relative, _inverse(reference))
            angle = 2.0 * math.atan2(float(delta[2]), float(delta[0]))
            angles.append(math.atan2(math.sin(angle), math.cos(angle)))
        return np.asarray(angles, dtype=float)

    @staticmethod
    def relative_zero(poses: dict) -> list[np.ndarray]:
        return [
            _multiply(_inverse(_quat(poses[parent])), _quat(poses[child]))
            for parent, child in JointAngleFeedback.LINKS
        ]

    def _on_pose(self, message) -> None:
        poses = {pose.name.rsplit('::', 1)[-1]: pose for pose in message.pose}
        if not all(parent in poses and child in poses for parent, child in self.LINKS):
            return
        with self._lock:
            if self._zero is None:
                if not self._calibration_allowed:
                    return
                self._zero = self.relative_zero(poses)
            angles = self.angles_from_poses(poses, self._zero)
            if np.all(np.isfinite(angles)):
                self._angles = angles
                self._received_at = time.monotonic()

    def stop_calibration(self) -> None:
        with self._lock:
            self._calibration_allowed = False

    def read(self, max_age_s: float = 0.12) -> tuple[np.ndarray | None, float]:
        with self._lock:
            age = time.monotonic() - self._received_at if self._received_at else math.inf
            if self._angles is None or age > max_age_s:
                return None, age
            return self._angles.copy(), age
