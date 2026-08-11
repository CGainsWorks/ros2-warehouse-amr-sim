"""Rigid-body transform helpers.

Written against numpy rather than pulled from ``tf_transformations`` for two
reasons: the analysis scripts have to run on recorded CSV logs with no ROS
installation present, and applying a transform to a 30k-point cloud by looping
through ``tf2_sensor_msgs`` is measurably slower than a single 3x3 matmul.

Quaternion order is ``(x, y, z, w)`` throughout, matching
``geometry_msgs/Quaternion``. Getting this backwards is the single most common
source of a silently mirrored point cloud, so every function that takes or
returns a quaternion says so in its signature.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a quaternion given as ``(x, y, z, w)``."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw (rotation about z) from a quaternion ``(x, y, z, w)``.

    Uses the standard atan2 form rather than a full Euler decomposition, which
    is both cheaper and free of the gimbal-lock branch. Valid for the small
    roll/pitch a ground robot sees.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion ``(x, y, z, w)`` for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def transform_points(
    points: np.ndarray,
    translation: np.ndarray,
    quaternion: Tuple[float, float, float, float],
) -> np.ndarray:
    """Apply ``p' = R p + t`` to an (N, 3) cloud."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3), got %r" % (points.shape,))
    if len(points) == 0:
        return points.copy()
    rot = quat_to_matrix(*quaternion)
    return points @ rot.T + np.asarray(translation, dtype=np.float64).reshape(3)


def wrap_angle(angle):
    """Wrap an angle, or array of angles, to ``[-pi, pi)``.

    Half-open at ``+pi``: an input of exactly ``pi`` comes back as ``-pi``.
    That is the standard behaviour of this formula and the two values are the
    same angle, so no branch is added to special-case it. It is documented
    because a test that asserts ``wrap_angle(3*pi) == pi`` will fail.
    """
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def se2_matrix(x: float, y: float, theta: float) -> np.ndarray:
    """3x3 homogeneous transform for a planar pose."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_from_matrix(m: np.ndarray) -> Tuple[float, float, float]:
    """Inverse of :func:`se2_matrix`."""
    return float(m[0, 2]), float(m[1, 2]), math.atan2(float(m[1, 0]), float(m[0, 0]))


def se2_inverse(x: float, y: float, theta: float) -> Tuple[float, float, float]:
    """Invert a planar pose without going via a matrix."""
    c, s = math.cos(theta), math.sin(theta)
    return (-(c * x + s * y), -(-s * x + c * y), -theta)


def se2_compose(a: Tuple[float, float, float], b: Tuple[float, float, float]):
    """Compose two planar poses, ``a`` then ``b``."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, float(wrap_angle(at + bt)))


def transform_points_2d(points_xy: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Apply a planar transform to an (N, 2) array."""
    points_xy = np.asarray(points_xy, dtype=np.float64)
    if len(points_xy) == 0:
        return points_xy.copy()
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return points_xy @ rot.T + np.array([x, y])
