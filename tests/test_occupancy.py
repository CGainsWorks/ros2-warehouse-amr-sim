"""3D voxel occupancy grid."""

from __future__ import annotations

import numpy as np
import pytest

from amr_perception.core.ground_removal import remove_ground
from amr_perception.core.occupancy_3d import build_occupancy


class TestBuildOccupancy:
    def test_single_dense_voxel(self):
        points = np.tile([0.05, 0.05, 0.05], (10, 1))
        grid = build_occupancy(points, resolution=0.1, min_points=2)
        assert grid.occupied_count == 1

    def test_min_points_suppresses_isolated_returns(self):
        """One stray return at grazing incidence must not become an obstacle."""
        dense = np.tile([0.05, 0.05, 0.05], (10, 1))
        stray = np.array([[5.05, 5.05, 0.05]])
        grid = build_occupancy(np.vstack([dense, stray]), resolution=0.1, min_points=2)
        assert grid.occupied_count == 1

    def test_min_points_one_keeps_everything(self):
        dense = np.tile([0.05, 0.05, 0.05], (10, 1))
        stray = np.array([[5.05, 5.05, 0.05]])
        grid = build_occupancy(np.vstack([dense, stray]), resolution=0.1, min_points=1)
        assert grid.occupied_count == 2

    def test_empty_cloud(self):
        grid = build_occupancy(np.empty((0, 3)), resolution=0.1)
        assert grid.occupied_count == 0

    def test_explicit_bounds_fix_the_grid_size(self):
        """A grid that resizes with the cloud has an origin that jitters in RViz."""
        bounds = (np.array([-5.0, -5.0, 0.0]), np.array([5.0, 5.0, 2.0]))
        rng = np.random.default_rng(0)
        for seed_offset in range(3):
            points = rng.uniform(-3, 3, size=(500, 3))
            points[:, 2] = np.abs(points[:, 2]) * 0.3
            grid = build_occupancy(points, resolution=0.1, bounds=bounds)
            assert grid.dims == (100, 100, 20)
            assert np.allclose(grid.origin, bounds[0])

    def test_points_outside_bounds_are_dropped_not_clamped(self):
        """Clamping would pile every distant return onto the boundary voxels."""
        bounds = (np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]))
        inside = np.tile([0.5, 0.5, 0.5], (5, 1))
        outside = np.tile([50.0, 50.0, 0.5], (100, 1))
        grid = build_occupancy(np.vstack([inside, outside]), resolution=0.1, bounds=bounds, min_points=1)
        assert grid.occupied_count == 1

    def test_invalid_bounds_rejected(self):
        with pytest.raises(ValueError):
            build_occupancy(np.zeros((5, 3)), bounds=(np.ones(3), np.zeros(3)))

    def test_invalid_resolution_rejected(self):
        with pytest.raises(ValueError):
            build_occupancy(np.zeros((5, 3)), resolution=0.0)

    def test_occupied_centres_land_inside_their_voxels(self):
        rng = np.random.default_rng(1)
        points = rng.uniform(0, 2, size=(300, 3))
        grid = build_occupancy(points, resolution=0.25, min_points=1)
        centres = grid.occupied_centres()
        assert len(centres) == grid.occupied_count
        assert np.all(centres >= grid.origin)


class TestProjection:
    def test_projection_collapses_height(self):
        column = np.column_stack([np.full(20, 0.05), np.full(20, 0.05), np.linspace(0.05, 1.9, 20)])
        grid = build_occupancy(column, resolution=0.1, min_points=1)
        footprint = grid.project_2d()
        assert footprint.sum() == 1

    def test_height_band_excludes_out_of_band_structure(self):
        low = np.tile([0.05, 0.05, 0.05], (5, 1))
        high = np.tile([1.05, 1.05, 2.55], (5, 1))
        bounds = (np.zeros(3), np.array([3.0, 3.0, 3.0]))
        grid = build_occupancy(np.vstack([low, high]), resolution=0.1, bounds=bounds, min_points=1)

        assert grid.project_2d().sum() == 2
        assert grid.project_2d(z_min=0.0, z_max=1.0).sum() == 1
        assert grid.project_2d(z_min=2.0, z_max=3.0).sum() == 1

    def test_empty_band_is_empty(self):
        points = np.tile([0.05, 0.05, 0.05], (5, 1))
        bounds = (np.zeros(3), np.array([1.0, 1.0, 3.0]))
        grid = build_occupancy(points, resolution=0.1, bounds=bounds, min_points=1)
        assert grid.project_2d(z_min=2.0, z_max=3.0).sum() == 0


class TestOnWarehouseScan:
    def test_aisle_centreline_is_free(self, scan):
        """The corridor the robot drives down must not be marked occupied."""
        _, object_mask, _ = remove_ground(scan.points)
        bounds = (np.array([-16.0, -7.0, 0.0]), np.array([16.0, 7.0, 3.0]))
        grid = build_occupancy(scan.points[object_mask], resolution=0.10, bounds=bounds, min_points=2)

        footprint = grid.project_2d(z_min=0.1, z_max=2.0)
        # A 5 m stretch of centreline ahead of the sensor at (-3, 0), stopping
        # short of x = 3.4 where the scene puts a pallet in the aisle. That
        # pallet is a genuine obstacle and must be marked; including it here
        # would be asserting that real obstacles are invisible.
        i0 = int((-2.0 - bounds[0][0]) / 0.10)
        i1 = int((3.0 - bounds[0][0]) / 0.10)
        j0 = int((-0.25 - bounds[0][1]) / 0.10)
        j1 = int((0.25 - bounds[0][1]) / 0.10)
        assert not footprint[i0:i1, j0:j1].any(), "the aisle centreline is marked occupied"

    def test_racking_appears_in_the_grid(self, scan):
        _, object_mask, _ = remove_ground(scan.points)
        bounds = (np.array([-16.0, -7.0, 0.0]), np.array([16.0, 7.0, 3.0]))
        grid = build_occupancy(scan.points[object_mask], resolution=0.10, bounds=bounds, min_points=2)
        footprint = grid.project_2d(z_min=0.1, z_max=2.0)

        # A band across the north rack face at y = 1.4.
        j0 = int((1.30 - bounds[0][1]) / 0.10)
        j1 = int((1.50 - bounds[0][1]) / 0.10)
        assert footprint[:, j0:j1].sum() > 5, "no racking found where it should be"
