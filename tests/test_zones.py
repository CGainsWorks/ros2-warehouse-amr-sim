"""Work-zone occupancy: geometry, debouncing, and the shipped zone file."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from amr_perception.core.clustering import cluster_points
from amr_perception.core.ground_removal import remove_ground
from amr_perception.core.synthetic import simulate_scan
from amr_perception.core.zones import (
    Zone,
    ZoneOccupancyTracker,
    ZoneState,
    evaluate_zone,
    evaluate_zones,
    load_zones,
    points_in_polygon,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
ZONE_FILE = REPO_ROOT / "simulation" / "zones" / "warehouse_zones.yaml"

SQUARE = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
# A concave polygon, to prove the even-odd test is not a convex-hull test.
L_SHAPE = np.array([[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]], dtype=float)


class TestPointInPolygon:
    def test_inside_and_outside(self):
        pts = np.array([[1.0, 1.0], [3.0, 1.0], [-1.0, 1.0], [1.0, 3.0]])
        assert points_in_polygon(pts, SQUARE).tolist() == [True, False, False, False]

    def test_concave_notch_is_excluded(self):
        pts = np.array([[0.5, 0.5], [2.0, 0.5], [2.0, 2.0], [0.5, 2.5]])
        result = points_in_polygon(pts, L_SHAPE).tolist()
        assert result == [True, True, False, True]

    def test_empty_input(self):
        assert len(points_in_polygon(np.empty((0, 2)), SQUARE)) == 0

    def test_horizontal_edge_does_not_divide_by_zero(self):
        """A test point level with a horizontal edge must not produce a nan."""
        pts = np.array([[1.0, 0.0], [1.0, 2.0]])
        result = points_in_polygon(pts, SQUARE)
        assert result.dtype == bool


class TestZone:
    def test_area_and_centroid(self):
        zone = Zone(name="z", polygon=SQUARE)
        assert zone.area == pytest.approx(4.0)
        assert zone.centroid == pytest.approx([1.0, 1.0])

    def test_degenerate_polygon_rejected(self):
        with pytest.raises(ValueError):
            Zone(name="z", polygon=np.array([[0.0, 0.0], [1.0, 1.0]]))

    def test_inverted_height_band_rejected(self):
        with pytest.raises(ValueError, match="z_max"):
            Zone(name="z", polygon=SQUARE, z_min=2.0, z_max=1.0)


class TestEvaluateZone:
    def test_counts_points_inside_the_prism(self):
        zone = Zone(name="z", polygon=SQUARE, z_min=0.2, z_max=1.8, min_points=5)
        inside = np.column_stack([np.full(20, 1.0), np.full(20, 1.0), np.linspace(0.3, 1.5, 20)])
        state = evaluate_zone(zone, inside)
        assert state.occupied
        assert state.point_count == 20

    def test_points_outside_the_height_band_do_not_count(self):
        zone = Zone(name="z", polygon=SQUARE, z_min=0.2, z_max=1.8, min_points=1)
        below = np.array([[1.0, 1.0, 0.05]])
        above = np.array([[1.0, 1.0, 2.5]])
        assert not evaluate_zone(zone, below).occupied
        assert not evaluate_zone(zone, above).occupied

    def test_min_points_threshold(self):
        zone = Zone(name="z", polygon=SQUARE, z_min=0.0, z_max=2.0, min_points=10)
        few = np.column_stack([np.ones(5), np.ones(5), np.full(5, 1.0)])
        assert not evaluate_zone(zone, few).occupied

    def test_object_at_the_very_edge_is_still_detected(self):
        """Counting points, not centroids, is what makes this work.

        A person half in and half out of a zone has a centroid outside it. A
        centroid-based test declares the zone clear, which is the wrong answer.
        """
        zone = Zone(name="z", polygon=SQUARE, z_min=0.0, z_max=2.0, min_points=5)
        rng = np.random.default_rng(0)
        # A blob straddling x = 2.0, the zone's right edge.
        blob = np.column_stack(
            [rng.normal(2.0, 0.15, 200), rng.normal(1.0, 0.15, 200), rng.uniform(0.2, 1.7, 200)]
        )
        assert blob[:, 0].mean() > 1.95  # centroid essentially on the boundary
        assert evaluate_zone(zone, blob).occupied

    def test_empty_cloud_is_clear(self):
        zone = Zone(name="z", polygon=SQUARE)
        state = evaluate_zone(zone, np.empty((0, 3)))
        assert not state.occupied and state.point_count == 0

    def test_reports_contributing_clusters(self):
        zone = Zone(name="z", polygon=SQUARE, z_min=0.0, z_max=2.0, min_points=5)
        rng = np.random.default_rng(1)
        inside = np.column_stack([rng.normal(1.0, 0.1, 300), rng.normal(1.0, 0.1, 300), rng.uniform(0.3, 1.5, 300)])
        outside = np.column_stack([rng.normal(9.0, 0.1, 300), rng.normal(9.0, 0.1, 300), rng.uniform(0.3, 1.5, 300)])
        points = np.vstack([inside, outside])
        clusters = cluster_points(points, min_points=25)
        state = evaluate_zone(zone, points, clusters)
        assert state.occupied
        assert len(state.cluster_ids) == 1


class TestTracker:
    def _states(self, occupied: bool):
        return [ZoneState(name="z", occupied=occupied, point_count=100 if occupied else 0)]

    def test_entering_takes_enter_frames(self):
        tracker = ZoneOccupancyTracker(enter_frames=2, exit_frames=5)
        assert tracker.update(self._states(True)).get("z", False) is False
        assert tracker.update(self._states(True))["z"] is True

    def test_leaving_takes_exit_frames(self):
        tracker = ZoneOccupancyTracker(enter_frames=2, exit_frames=5)
        for _ in range(2):
            tracker.update(self._states(True))
        assert tracker.latched["z"] is True
        for _ in range(4):
            assert tracker.update(self._states(False))["z"] is True
        assert tracker.update(self._states(False))["z"] is False

    def test_single_dropped_frame_does_not_clear(self):
        """The whole reason the hysteresis is asymmetric."""
        tracker = ZoneOccupancyTracker(enter_frames=2, exit_frames=5)
        for _ in range(2):
            tracker.update(self._states(True))
        tracker.update(self._states(False))  # one dropped frame
        tracker.update(self._states(True))
        assert tracker.latched["z"] is True

    def test_streak_resets_on_agreement(self):
        tracker = ZoneOccupancyTracker(enter_frames=3, exit_frames=3)
        tracker.update(self._states(True))
        tracker.update(self._states(True))
        tracker.update(self._states(False))  # agrees with the latched False, resets
        tracker.update(self._states(True))
        tracker.update(self._states(True))
        assert tracker.latched.get("z", False) is False

    def test_invalid_frame_counts_rejected(self):
        with pytest.raises(ValueError):
            ZoneOccupancyTracker(enter_frames=0, exit_frames=5)

    def test_reset(self):
        tracker = ZoneOccupancyTracker(enter_frames=1, exit_frames=1)
        tracker.update(self._states(True))
        tracker.reset()
        assert tracker.latched == {}


class TestShippedZoneFile:
    def test_loads(self):
        zones = load_zones(str(ZONE_FILE))
        assert len(zones) == 7
        assert {z.name for z in zones} >= {"charger_bay", "pick_face_north_0"}

    def test_pick_faces_clear_the_racking(self):
        """No pick-face zone may contain its own rack structure.

        A zone overlapping the racking is permanently occupied by returns from
        the uprights, so it can never report a person. This is the single most
        likely way to get a plausible-looking but useless zone configuration.
        The uprights' inner faces are at |y| = 1.35.
        """
        for zone in load_zones(str(ZONE_FILE)):
            if not zone.name.startswith("pick_face"):
                continue
            assert np.abs(zone.polygon[:, 1]).max() <= 1.30, (
                "%s reaches y=%.2f and so overlaps the racking at 1.35"
                % (zone.name, np.abs(zone.polygon[:, 1]).max())
            )

    def test_zones_are_clear_of_static_structure_in_a_real_scan(self, scene):
        """With no dynamic objects present, no pick face may read occupied."""
        static = type(scene)()
        groups = {c.instance for c in scene.cylinders if c.label == 3}
        for box in scene.boxes:
            static.add_box(box.lo, box.hi, box.label)
        for cyl in scene.cylinders:
            if cyl.instance not in groups:  # drop the people
                static.add_cylinder(cyl.centre_xy, cyl.radius, cyl.z_lo, cyl.z_hi, cyl.label)

        zones = [z for z in load_zones(str(ZONE_FILE)) if z.name.startswith("pick_face")]
        for sensor_x in (-9.45, -1.35, 6.75):
            scan = simulate_scan(static, sensor_xyz=(sensor_x, 0.0, 0.9), seed=3)
            _, object_mask, _ = remove_ground(scan.points)
            points = scan.points[object_mask]
            for state in evaluate_zones(zones, points):
                # The aisle pallets are legitimately inside two pick faces, so
                # only zones with no aisle obstacle in them are asserted clear.
                if state.name in ("pick_face_north_2", "pick_face_south_0"):
                    continue
                assert not state.occupied, (
                    "%s reads occupied from x=%.2f with only static structure present (%d points)"
                    % (state.name, sensor_x, state.point_count)
                )

    def test_person_at_a_pick_face_is_detected(self, scene):
        """The positive case: a person in the aisle strip latches the zone."""
        zones = {z.name: z for z in load_zones(str(ZONE_FILE))}
        scan = simulate_scan(scene, sensor_xyz=(-4.5, 0.0, 0.9), seed=4)
        _, object_mask, _ = remove_ground(scan.points)
        points = scan.points[object_mask]
        # The scene puts a person at (-3.0, 1.0), inside pick_face_north_1.
        state = evaluate_zone(zones["pick_face_north_1"], points)
        assert state.occupied, "person at (-3, 1.0) not detected, %d points" % state.point_count
