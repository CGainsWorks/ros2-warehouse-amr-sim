"""Euclidean clustering and the voxel front end."""

from __future__ import annotations

import numpy as np
import pytest

from amr_perception.core.clustering import cluster_label_array, cluster_points, connected_components
from amr_perception.core.ground_removal import remove_ground
from amr_perception.core.voxel import pack_indices, voxel_downsample, voxel_indices


def _blob(centre, n=200, radius=0.15, seed=0):
    rng = np.random.default_rng(seed)
    return np.asarray(centre) + rng.normal(0, radius, size=(n, 3))


class TestConnectedComponents:
    def test_no_edges_gives_one_component_per_node(self):
        labels = connected_components(5, np.empty((0, 2), dtype=np.int64))
        assert len(set(labels.tolist())) == 5

    def test_chain_is_one_component(self):
        pairs = np.array([[0, 1], [1, 2], [2, 3], [3, 4]])
        labels = connected_components(5, pairs)
        assert len(set(labels.tolist())) == 1

    def test_two_disjoint_groups(self):
        pairs = np.array([[0, 1], [2, 3]])
        labels = connected_components(4, pairs)
        assert labels[0] == labels[1]
        assert labels[2] == labels[3]
        assert labels[0] != labels[2]

    def test_long_chain_does_not_blow_the_stack(self):
        """A 20 m run of racking is one component thousands of nodes long."""
        n = 50_000
        pairs = np.stack([np.arange(n - 1), np.arange(1, n)], axis=1)
        labels = connected_components(n, pairs)
        assert len(set(labels.tolist())) == 1


class TestVoxel:
    def test_downsample_reduces_and_maps_back(self):
        points = _blob([0, 0, 0], n=1000, radius=0.5)
        centroids, inverse, counts = voxel_downsample(points, 0.1)
        assert len(centroids) < len(points)
        assert len(inverse) == len(points)
        assert counts.sum() == len(points)
        assert inverse.max() == len(centroids) - 1

    def test_centroid_is_the_mean_of_its_members(self):
        points = _blob([1, 2, 3], n=500, radius=0.3, seed=2)
        centroids, inverse, _ = voxel_downsample(points, 0.15)
        for index in range(len(centroids)):
            members = points[inverse == index]
            assert np.allclose(centroids[index], members.mean(axis=0))

    def test_empty_input(self):
        centroids, inverse, counts = voxel_downsample(np.empty((0, 3)), 0.1)
        assert len(centroids) == 0 and len(inverse) == 0 and len(counts) == 0

    def test_zero_resolution_rejected(self):
        with pytest.raises(ValueError):
            voxel_indices(np.zeros((3, 3)), 0.0)

    def test_packing_overflow_is_reported_not_silent(self):
        """Aliasing two distant voxels onto one key must raise, not corrupt."""
        idx = np.array([[0, 0, 0], [10**7, 10**7, 10**7]], dtype=np.int64)
        with pytest.raises(ValueError, match="overflow"):
            pack_indices(idx)


class TestClustering:
    def test_two_separated_blobs_are_two_clusters(self):
        points = np.vstack([_blob([0, 0, 0], seed=1), _blob([5, 0, 0], seed=2)])
        clusters = cluster_points(points, tolerance=0.35, min_points=20)
        assert len(clusters) == 2

    def test_touching_blobs_are_one_cluster(self):
        """Connectivity is the definition; this is correct behaviour, not a bug."""
        points = np.vstack([_blob([0, 0, 0], seed=1), _blob([0.2, 0, 0], seed=2)])
        clusters = cluster_points(points, tolerance=0.35, min_points=20)
        assert len(clusters) == 1

    def test_min_points_filters_small_clusters(self):
        points = np.vstack([_blob([0, 0, 0], n=500, seed=1), _blob([5, 0, 0], n=5, seed=2)])
        clusters = cluster_points(points, min_points=25)
        assert len(clusters) == 1

    def test_min_height_filters_flat_clutter(self):
        rng = np.random.default_rng(0)
        flat = np.column_stack([rng.uniform(-0.3, 0.3, 300), rng.uniform(-0.3, 0.3, 300), rng.uniform(0, 0.03, 300)])
        tall = _blob([5, 0, 1.0], n=300, seed=3)
        clusters = cluster_points(np.vstack([flat, tall]), min_points=25, min_height=0.10)
        assert len(clusters) == 1
        assert clusters[0].centroid[0] == pytest.approx(5.0, abs=0.2)

    def test_voxel_size_must_be_under_tolerance(self):
        """Guarding this is the difference between a bug and an exception."""
        with pytest.raises(ValueError, match="voxel_size"):
            cluster_points(_blob([0, 0, 0]), tolerance=0.2, voxel_size=0.25)

    def test_indices_are_into_the_input_array(self):
        points = np.vstack([_blob([0, 0, 0], seed=1), _blob([5, 0, 0], seed=2)])
        clusters = cluster_points(points, min_points=20)
        for cluster in clusters:
            assert cluster.indices.max() < len(points)
            assert np.allclose(cluster.centroid, points[cluster.indices].mean(axis=0))
            assert np.allclose(cluster.bbox_min, points[cluster.indices].min(axis=0))

    def test_clusters_are_disjoint_and_sorted_by_size(self):
        points = np.vstack([_blob([0, 0, 0], n=400, seed=1), _blob([5, 0, 0], n=100, seed=2)])
        clusters = cluster_points(points, min_points=20)
        sizes = [c.size for c in clusters]
        assert sizes == sorted(sizes, reverse=True)

        seen = set()
        for cluster in clusters:
            member = set(cluster.indices.tolist())
            assert not (seen & member), "a point was assigned to two clusters"
            seen |= member

    def test_ids_are_contiguous_from_zero(self):
        points = np.vstack([_blob([0, 0, 0], seed=1), _blob([5, 0, 0], seed=2), _blob([10, 0, 0], seed=3)])
        clusters = cluster_points(points, min_points=20)
        assert [c.id for c in clusters] == list(range(len(clusters)))

    def test_empty_input(self):
        assert cluster_points(np.empty((0, 3))) == []

    def test_label_array_marks_unassigned_as_minus_one(self):
        points = np.vstack([_blob([0, 0, 0], n=400, seed=1), _blob([5, 0, 0], n=3, seed=2)])
        clusters = cluster_points(points, min_points=25)
        labels = cluster_label_array(points, clusters)
        assert (labels == -1).sum() == 3


class TestOnWarehouseScan:
    def test_people_in_the_open_aisle_are_found_separately(self, scan, scene):
        """A person standing clear of the racking must be its own cluster.

        The person here is about 11 m away, which is past the range at which a
        fixed 0.35 m tolerance holds a person together across LiDAR rings. This
        test therefore only passes because of the range-adaptive merge; it is
        the regression guard for that behaviour.
        """
        _, object_mask, _ = remove_ground(scan.points)
        points = scan.points[object_mask]
        instances = scan.instances[object_mask]
        clusters = cluster_points(
            points, min_points=25, min_height=0.10, sensor_origin=tuple(scan.sensor_xyz)
        )

        # The person at (8, 0) is 1.1 m from the nearest rack face, far beyond
        # the 0.35 m tolerance, so nothing may merge with them.
        person = [c for c in scene.cylinders if c.label == 3 and abs(c.centre_xy[0] - 8.0) < 0.1]
        assert person, "expected a person at x=8 in the scene"
        member = instances == person[0].instance
        if member.sum() < 25:
            import pytest as _pytest

            _pytest.skip("person at x=8 not sufficiently visible from this viewpoint")

        overlaps = [c for c in clusters if member[c.indices].any()]
        assert overlaps, "the person was not detected at all"
        best = max(overlaps, key=lambda c: member[c.indices].sum())
        captured = member[best.indices].sum() / member.sum()
        assert captured > 0.8, "the person was split across clusters"

    def test_no_cluster_spans_the_aisle_within_the_racked_section(self, scan):
        """The robot's corridor must never be declared one obstacle.

        Restricted to |x| < 13.5, the extent of the racking. The end walls at
        x = +/-15 legitimately span the full width of the building and appear as
        one cluster crossing the aisle centreline, which is correct — they are
        one wall. Asserting over the whole scene would fail on that and would be
        testing the wrong thing.
        """
        _, object_mask, _ = remove_ground(scan.points)
        clusters = cluster_points(
            scan.points[object_mask], min_points=25, sensor_origin=tuple(scan.sensor_xyz)
        )
        for cluster in clusters:
            if cluster.bbox_min[0] < -13.5 or cluster.bbox_max[0] > 13.5:
                continue  # touches an end wall
            spans_north = cluster.bbox_max[1] > 1.3
            spans_south = cluster.bbox_min[1] < -1.3
            assert not (spans_north and spans_south), (
                "cluster %d bridges the aisle: y from %.2f to %.2f"
                % (cluster.id, cluster.bbox_min[1], cluster.bbox_max[1])
            )
