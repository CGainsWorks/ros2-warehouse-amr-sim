"""Rigid-body transform helpers.

Quaternion order is (x, y, z, w) throughout. Most of these tests exist to pin
that down, because a silently transposed rotation or a swapped w produces a
cloud that looks almost right in RViz and is completely wrong.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from amr_common.transforms import (
    quat_from_yaw,
    quat_to_matrix,
    se2_compose,
    se2_from_matrix,
    se2_inverse,
    se2_matrix,
    transform_points,
    transform_points_2d,
    wrap_angle,
    yaw_from_quat,
)


class TestQuaternions:
    def test_identity(self):
        assert np.allclose(quat_to_matrix(0, 0, 0, 1), np.eye(3))

    def test_yaw_round_trip(self):
        for yaw in (-3.0, -1.0, 0.0, 0.5, 1.57, 3.0):
            assert yaw_from_quat(*quat_from_yaw(yaw)) == pytest.approx(yaw, abs=1e-9)

    def test_rotation_is_not_transposed(self):
        """A 90 degree yaw must take +x to +y, not to -y."""
        rot = quat_to_matrix(*quat_from_yaw(math.pi / 2))
        assert np.allclose(rot @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)

    def test_matrix_is_orthonormal(self):
        rot = quat_to_matrix(0.2, -0.3, 0.1, 0.9)
        assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-9)
        assert np.linalg.det(rot) == pytest.approx(1.0)

    def test_unnormalised_input_is_normalised(self):
        rot = quat_to_matrix(0.0, 0.0, 2.0, 2.0)  # 90 degrees, unnormalised
        assert np.allclose(rot @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)

    def test_zero_quaternion_rejected(self):
        with pytest.raises(ValueError):
            quat_to_matrix(0.0, 0.0, 0.0, 0.0)

    def test_yaw_survives_pitch(self):
        """Ground robots pitch under acceleration; yaw extraction must not care."""
        pitch, yaw = math.radians(10), math.radians(40)
        cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        # Hamilton product q_yaw (x) q_pitch, in (x, y, z, w) order. This is the
        # ZYX convention the atan2 extraction assumes, so the recovered yaw
        # should be exact rather than approximate.
        q = (-sz * sp, cz * sp, sz * cp, cz * cp)
        assert yaw_from_quat(*q) == pytest.approx(yaw, abs=1e-9)


class TestTransformPoints:
    def test_translation_only(self):
        points = np.array([[1.0, 2.0, 3.0]])
        moved = transform_points(points, np.array([1.0, 1.0, 1.0]), (0, 0, 0, 1))
        assert np.allclose(moved, [[2.0, 3.0, 4.0]])

    def test_rotation_then_translation_order(self):
        """p' = R p + t, not R (p + t)."""
        points = np.array([[1.0, 0.0, 0.0]])
        moved = transform_points(points, np.array([5.0, 0.0, 0.0]), quat_from_yaw(math.pi / 2))
        assert np.allclose(moved, [[5.0, 1.0, 0.0]], atol=1e-9)

    def test_empty_cloud(self):
        assert transform_points(np.empty((0, 3)), np.zeros(3), (0, 0, 0, 1)).shape == (0, 3)

    def test_wrong_shape_raises(self):
        with pytest.raises(ValueError):
            transform_points(np.zeros((4, 2)), np.zeros(3), (0, 0, 0, 1))

    def test_preserves_distances(self):
        rng = np.random.default_rng(0)
        points = rng.normal(size=(50, 3))
        moved = transform_points(points, np.array([1.0, -2.0, 0.5]), (0.1, 0.2, 0.3, 0.9))
        before = np.linalg.norm(points[1:] - points[:-1], axis=1)
        after = np.linalg.norm(moved[1:] - moved[:-1], axis=1)
        assert np.allclose(before, after)


class TestSE2:
    def test_compose_is_a_then_b_in_a_frame(self):
        """se2_compose(a, b) applies b in a's frame."""
        a = (1.0, 0.0, math.pi / 2)
        b = (1.0, 0.0, 0.0)  # 1 m forward in a's frame, which points at +y
        result = se2_compose(a, b)
        assert result[0] == pytest.approx(1.0, abs=1e-9)
        assert result[1] == pytest.approx(1.0, abs=1e-9)

    def test_inverse_round_trip(self):
        pose = (2.5, -1.0, 0.7)
        identity = se2_compose(pose, se2_inverse(*se2_compose(pose, (0.0, 0.0, 0.0))))
        assert identity[0] == pytest.approx(0.0, abs=1e-9)
        assert identity[1] == pytest.approx(0.0, abs=1e-9)
        assert identity[2] == pytest.approx(0.0, abs=1e-9)

    def test_relative_pose_recovers_motion(self):
        a = (-6.0, 0.3, 0.2)
        motion = (0.4, -0.1, 0.15)
        b = se2_compose(a, motion)
        recovered = se2_compose(se2_inverse(*a), b)
        assert np.allclose(recovered, motion, atol=1e-9)

    def test_matrix_round_trip(self):
        pose = (1.2, -3.4, 0.9)
        assert np.allclose(se2_from_matrix(se2_matrix(*pose)), pose)

    def test_matrix_composition_matches_se2_compose(self):
        a = (1.0, 2.0, 0.3)
        b = (0.5, -0.2, -0.7)
        assert np.allclose(se2_from_matrix(se2_matrix(*a) @ se2_matrix(*b)), se2_compose(a, b))

    def test_transform_points_2d(self):
        points = np.array([[1.0, 0.0]])
        moved = transform_points_2d(points, 0.0, 0.0, math.pi / 2)
        assert np.allclose(moved, [[0.0, 1.0]], atol=1e-9)


class TestWrapAngle:
    def test_scalar(self):
        assert wrap_angle(0.5) == pytest.approx(0.5)
        assert wrap_angle(2 * math.pi + 0.5) == pytest.approx(0.5)
        assert wrap_angle(-2 * math.pi + 0.5) == pytest.approx(0.5)

    def test_range_is_half_open_at_plus_pi(self):
        """Documented behaviour: exactly pi comes back as -pi."""
        assert wrap_angle(math.pi) == pytest.approx(-math.pi)
        assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)

    def test_array(self):
        angles = np.array([0.0, 7.0, -7.0, math.pi])
        wrapped = wrap_angle(angles)
        assert np.all(wrapped >= -math.pi - 1e-9)
        assert np.all(wrapped < math.pi + 1e-9)
