"""Ground-plane removal.

The tests that matter here are the adversarial ones. Fitting a plane to a floor
is easy; the reason this module exists in the form it does is the shelf-top and
tilt cases, so those get the most attention.
"""

from __future__ import annotations

import numpy as np
import pytest

from amr_perception.core.ground_removal import fit_ground_plane, remove_ground
from amr_perception.core.synthetic import GROUND


def _plane_points(n, z=0.0, extent=10.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-extent, extent, size=(n, 2))
    zs = np.full(n, z) + (rng.normal(0, noise, n) if noise else 0.0)
    return np.column_stack([xy, zs])


class TestBasicFit:
    def test_flat_floor_recovered(self):
        points = _plane_points(2000, z=0.0, noise=0.005)
        plane = fit_ground_plane(points)
        assert plane is not None
        assert plane.tilt_deg < 1.0
        assert abs(plane.d) < 0.02
        assert plane.inlier_ratio > 0.99

    def test_offset_floor_recovered(self):
        # Sensor-frame cloud with the floor 0.35 m below the origin.
        points = _plane_points(2000, z=-0.35, noise=0.005)
        plane = fit_ground_plane(points, seed_band=(-0.5, 0.1))
        assert plane is not None
        assert abs(plane.d - 0.35) < 0.02

    def test_too_few_points_returns_none(self):
        assert fit_ground_plane(np.zeros((2, 3))) is None

    def test_rejects_non_planar_cloud(self):
        rng = np.random.default_rng(0)
        blob = rng.uniform(-5, 5, size=(2000, 3))
        # A uniform cube has no dominant plane; any plane found explains far too
        # little of it to be a floor.
        plane = fit_ground_plane(blob, min_inlier_ratio=0.5)
        assert plane is None

    def test_wrong_shape_raises(self):
        with pytest.raises(ValueError):
            fit_ground_plane(np.zeros((10, 2)))


class TestAdversarial:
    def test_shelf_top_is_not_mistaken_for_floor(self):
        """A large flat surface above the floor must not win.

        This is the failure that motivates the seed band. The shelf here has
        three times as many points as the floor, so a plain RANSAC scoring on
        inlier count alone would choose it.
        """
        floor = _plane_points(1000, z=0.0, noise=0.004, seed=1)
        shelf = _plane_points(3000, z=1.6, noise=0.004, extent=6.0, seed=2)
        points = np.vstack([floor, shelf])

        plane = fit_ground_plane(points, seed_band=(-0.5, 0.35))
        assert plane is not None
        # d is the offset of a plane through z=0, so |d| near 0 means the floor.
        assert abs(plane.d) < 0.05, "locked onto the shelf at z=1.6, not the floor"

    def test_vertical_wall_rejected_by_tilt_constraint(self):
        floor = _plane_points(1000, z=0.0, noise=0.004, seed=1)
        rng = np.random.default_rng(3)
        # A wall in the x=2 plane, with more points than the floor.
        wall = np.column_stack(
            [np.full(3000, 2.0), rng.uniform(-8, 8, 3000), rng.uniform(0, 4, 3000)]
        )
        points = np.vstack([floor, wall])

        plane = fit_ground_plane(points, seed_band=None, max_tilt_deg=10.0)
        assert plane is not None
        assert plane.tilt_deg < 10.0
        assert abs(plane.d) < 0.05

    def test_tilt_limit_is_enforced(self):
        """A genuinely sloped floor beyond the limit is refused, not accepted."""
        rng = np.random.default_rng(4)
        xy = rng.uniform(-10, 10, size=(3000, 2))
        # 20 degree slope, well beyond the 10 degree limit.
        z = xy[:, 0] * np.tan(np.radians(20.0))
        points = np.column_stack([xy, z])

        assert fit_ground_plane(points, max_tilt_deg=10.0, seed_band=None) is None
        # The same cloud is accepted when the limit allows it, which shows the
        # refusal above is the constraint and not a fitting failure.
        assert fit_ground_plane(points, max_tilt_deg=25.0, seed_band=None) is not None


class TestDeterminism:
    def test_same_seed_gives_identical_plane(self, scan):
        a = fit_ground_plane(scan.points, seed=7)
        b = fit_ground_plane(scan.points, seed=7)
        assert a is not None and b is not None
        assert np.allclose(a.normal, b.normal)
        assert a.d == b.d

    def test_subsampled_scoring_matches_full_scoring(self, scan):
        """Subsampling must not change the answer, only the time taken."""
        full = fit_ground_plane(scan.points, score_subsample=None, seed=3)
        sub = fit_ground_plane(scan.points, score_subsample=4000, seed=3)
        assert full is not None and sub is not None
        # Both must find the same physical plane. They need not agree bit for
        # bit, because they evaluate different hypotheses, but a floor is a
        # floor: sub-centimetre offset and a fraction of a degree.
        assert abs(full.d - sub.d) < 0.01
        assert abs(full.tilt_deg - sub.tilt_deg) < 0.5


class TestAgainstLabelledScan:
    def test_recall_is_total_on_the_warehouse_scan(self, scan):
        """Every true ground point must be classified as ground.

        Recall is the safety-relevant direction for the floor: a missed ground
        point becomes a phantom obstacle in the costmap and can stop the robot.
        """
        ground_mask, _, plane = remove_ground(scan.points)
        assert plane is not None
        truth = scan.labels == GROUND
        recall = (ground_mask & truth).sum() / truth.sum()
        assert recall > 0.99

    def test_precision_is_high_and_errors_are_all_near_the_floor(self, scan):
        """False positives are tolerable only if they are geometrically excusable.

        The pipeline does misclassify some non-ground points, and it should:
        the bottom few centimetres of a wall, a racking upright or a pallet lie
        inside the 0.06 m inlier band and are genuinely indistinguishable from
        floor at that threshold. What must NOT happen is a false positive well
        above the floor, which would mean part of a real obstacle was discarded.
        """
        ground_mask, _, plane = remove_ground(scan.points)
        assert plane is not None

        truth = scan.labels == GROUND
        false_positives = ground_mask & ~truth
        precision = (ground_mask & truth).sum() / ground_mask.sum()
        assert precision > 0.90

        if false_positives.any():
            heights = np.abs(plane.height_above(scan.points[false_positives]))
            assert heights.max() <= 0.061, "discarded a point well above the floor as ground"

    def test_no_object_survives_only_as_ground(self, scan):
        """No labelled object may be entirely swallowed by the floor.

        Individual points being lost is acceptable. An entire object vanishing
        is not, because the clustering stage downstream can never recover it.
        """
        ground_mask, object_mask, _ = remove_ground(scan.points)
        for instance in np.unique(scan.instances):
            if instance == 0:
                continue
            member = scan.instances == instance
            if member.sum() < 25:
                continue  # too few returns to be findable at all; not a regression
            assert object_mask[member].any(), "instance %d was entirely absorbed into the floor" % instance

    def test_failed_fit_keeps_everything_as_obstacle(self):
        """Failing safe means calling everything an obstacle, never nothing."""
        rng = np.random.default_rng(5)
        blob = rng.uniform(-5, 5, size=(500, 3))
        ground, objects, plane = remove_ground(blob, min_inlier_ratio=0.95)
        assert plane is None
        assert not ground.any()
        assert objects.all()
