"""Euclidean clustering of the non-ground returns.

This is the classic PCL ``EuclideanClusterExtraction`` algorithm — connected
components under a radius graph — with two changes that matter in a warehouse:

* Clustering runs on a voxel-downsampled cloud, but the size filters are applied
  in *original* point counts. A pallet 2 m away and the same pallet 15 m away
  differ by an order of magnitude in raw return count; downsampling first makes
  the connectivity behave the same at both ranges, while filtering on raw counts
  keeps "how much evidence is there for this object" honest.
* The connection radius grows with range, because a fixed radius cannot work.
  A 16-ring sensor spanning 30 degrees has 2 degrees between rings, so on a
  vertical surface the rings are ``0.035 * range`` apart: 0.17 m at 5 m, but
  0.42 m at 12 m. Any fixed tolerance is therefore a range limit in disguise —
  at 0.35 m, objects past about 10 m fall apart into one cluster per ring, each
  too small to survive ``min_points``, and a person at the far end of the aisle
  is detected as nothing at all. Raising the fixed tolerance to cover 15 m
  instead merges a person standing near a rack face into the racking, which is
  worse. The fix is to keep the tolerance tight where the data is dense and let
  it grow only as far as the ring geometry requires.
* The neighbour graph is built with a single vectorised ``query_pairs`` call
  and labelled by ``scipy.sparse.csgraph``, both of which stay in C. An
  equivalent union-find written in Python is correct but spends around 100 ms
  per scan walking a half-million-edge list, which on its own blows the 10 Hz
  budget. The graph is also why a recursive flood fill is not used: a 20 m run
  of racking is one connected component thousands of nodes long.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components as _csgraph_components
from scipy.spatial import cKDTree

from .voxel import voxel_downsample


@dataclass
class Cluster:
    """One connected component of the non-ground cloud."""

    id: int
    indices: np.ndarray  # indices into the cloud passed to cluster_points()
    centroid: np.ndarray
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    extras: dict = field(default_factory=dict)

    @property
    def size(self) -> int:
        return int(len(self.indices))

    @property
    def dimensions(self) -> np.ndarray:
        return self.bbox_max - self.bbox_min

    @property
    def footprint_area(self) -> float:
        dims = self.dimensions
        return float(dims[0] * dims[1])

    @property
    def height(self) -> float:
        return float(self.dimensions[2])

    def as_dict(self) -> dict:
        return {
            "id": int(self.id),
            "size": self.size,
            "centroid": [float(v) for v in self.centroid],
            "bbox_min": [float(v) for v in self.bbox_min],
            "bbox_max": [float(v) for v in self.bbox_max],
            "footprint_area": self.footprint_area,
            "height": self.height,
        }


def connected_components(n: int, pairs: np.ndarray) -> np.ndarray:
    """Label ``n`` nodes given an (E, 2) array of undirected edges.

    Edge weights are all 1 rather than the actual distances: a stored zero would
    be indistinguishable from a missing entry in a sparse matrix, and two voxel
    centroids that happen to coincide would then silently lose their edge.
    """
    if n == 0:
        return np.empty(0, dtype=np.int64)
    if len(pairs) == 0:
        return np.arange(n, dtype=np.int64)

    rows = pairs[:, 0]
    cols = pairs[:, 1]
    data = np.ones(len(pairs), dtype=np.int8)
    graph = coo_matrix((data, (rows, cols)), shape=(n, n)).tocsr()

    _, labels = _csgraph_components(graph, directed=False, return_labels=True)
    return labels.astype(np.int64)


def _bbox_gap(lo_a: np.ndarray, hi_a: np.ndarray, lo_b: np.ndarray, hi_b: np.ndarray) -> np.ndarray:
    """Euclidean gap between two axis-aligned boxes, 0 if they overlap."""
    gap = np.maximum(0.0, np.maximum(lo_b - hi_a, lo_a - hi_b))
    return np.linalg.norm(gap, axis=-1)


def _merge_by_range(
    groups: List[np.ndarray],
    points: np.ndarray,
    tolerance: float,
    ring_spacing_rad: float,
    range_gain: float,
    sensor_origin: np.ndarray,
    max_tolerance: float,
) -> List[np.ndarray]:
    """Join fragments of one object that the fixed tolerance left separated.

    Rather than running the whole radius graph again with a per-point radius —
    which scipy's ``query_pairs`` cannot do and which would cost far more — this
    works on the components already found. Ring fragments of a single object are
    a handful of stacked slices at essentially the same range, so comparing
    component bounding boxes is both sufficient and cheap.

    The tolerance for a pair uses the *nearer* of the two components. That is
    the conservative choice: two components at genuinely different ranges get
    the tighter radius, so a near object cannot reach out and absorb a far one.
    """
    if len(groups) < 2:
        return groups

    lo = np.array([points[g].min(axis=0) for g in groups])
    hi = np.array([points[g].max(axis=0) for g in groups])
    centroid = np.array([points[g].mean(axis=0) for g in groups])
    ranges = np.linalg.norm(centroid[:, :2] - sensor_origin[:2], axis=1)

    tol = np.clip(range_gain * ring_spacing_rad * ranges, tolerance, max_tolerance)

    # Only pairs whose centroids are close enough to possibly qualify are
    # tested. The search radius allows for the largest tolerance plus the
    # half-diagonals of the two boxes.
    half_diag = np.linalg.norm(hi - lo, axis=1) / 2.0
    search = float(max_tolerance + 2.0 * half_diag.max())
    tree = cKDTree(centroid)
    candidates = tree.query_pairs(r=search, output_type="ndarray")
    if len(candidates) == 0:
        return groups

    a = candidates[:, 0]
    b = candidates[:, 1]
    gap = _bbox_gap(lo[a], hi[a], lo[b], hi[b])
    joined = gap <= np.minimum(tol[a], tol[b])
    edges = candidates[joined]
    if len(edges) == 0:
        return groups

    labels = connected_components(len(groups), edges)
    merged: List[np.ndarray] = []
    for label in range(labels.max() + 1):
        members = [groups[i] for i in np.flatnonzero(labels == label)]
        merged.append(np.sort(np.concatenate(members)) if len(members) > 1 else members[0])
    return merged


def cluster_points(
    points: np.ndarray,
    tolerance: float = 0.35,
    min_points: int = 25,
    max_points: int = 200_000,
    voxel_size: float = 0.12,
    min_height: float = 0.0,
    range_adaptive: bool = True,
    ring_spacing_rad: float = 0.0349,
    range_gain: float = 1.3,
    max_tolerance: float = 0.80,
    sensor_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> List[Cluster]:
    """Cluster ``points`` into objects.

    Parameters
    ----------
    tolerance
        Radius, in metres, at which two points are considered connected.
        0.35 m is chosen against the two failure modes it sits between. The
        warehouse aisle is 2.4 m wide and racking uprights are 0.9 m apart
        within a bay, so anything below ~0.5 m avoids merging adjacent uprights
        into one object. At the other end, the 3D LiDAR's 16 rings give a
        vertical spacing of about 0.30 m on a 2 m tall object at 12 m range, so
        below ~0.32 m a person at the far end of the aisle fragments into one
        cluster per ring. 0.35 m clears the fragmentation limit with a little
        margin and stays well under the merge limit.
    min_points
        Minimum raw returns for a cluster to be reported. 25 is roughly the
        return count of a 0.3 m wide object at the 15 m design range; below that
        the cluster's extent is too poorly determined to be worth publishing,
        and the count is dominated by noise.
    max_points
        Guard against a failed ground fit turning the whole floor into one
        gigantic cluster. Effectively disabled by default; the zone publisher
        sets it much lower.
    voxel_size
        Downsampling resolution for the connectivity graph. The binding
        constraint is that the voxel *diagonal* must stay below ``tolerance``,
        or downsampling severs connections the clustering is meant to find:
        at 0.12 m the diagonal is 0.21 m against a 0.35 m tolerance. Going
        finer is not free — the radius-graph edge count grows with roughly the
        sixth power of ``tolerance / voxel_size``, and 0.08 m costs about an
        order of magnitude more time than 0.12 m for identical cluster output
        on this scene.
    min_height
        Discard clusters shorter than this. Left at 0 here; the zone publisher
        raises it to ignore floor debris and tape markings.
    range_adaptive
        Merge components that the fixed ``tolerance`` split apart purely because
        of ring divergence at range. See the module docstring.
    ring_spacing_rad
        Angular gap between adjacent LiDAR rings. 0.0349 rad is 2 degrees, from
        the 16 rings over 30 degrees configured in ``amr_description``. This is
        a property of the sensor and must be changed with it.
    range_gain
        Multiplier on the predicted ring spacing. 1.3 gives 30% headroom for
        surfaces that are not exactly vertical, where the spacing is larger than
        the vertical-surface formula predicts.
    max_tolerance
        Hard ceiling on the adaptive radius, 0.80 m. Without it the radius keeps
        growing with range and eventually exceeds the 1.1 m clearance between
        the aisle centreline and a rack face, at which point a person at the far
        end of the aisle merges into the racking — which is precisely the
        outcome the adaptation exists to avoid. At 0.80 m the radius stops
        growing at about 17.6 m.
    sensor_origin
        Where range is measured from, in the same frame as ``points``. The
        default of the origin is correct for a cloud in ``base_footprint``.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3), got %r" % (points.shape,))
    if voxel_size > 0 and voxel_size >= tolerance:
        raise ValueError(
            "voxel_size (%r) must be smaller than tolerance (%r) or downsampling "
            "will sever connections the clustering is meant to find"
            % (voxel_size, tolerance)
        )
    if len(points) == 0:
        return []

    if voxel_size > 0:
        nodes, inverse, _ = voxel_downsample(points, voxel_size)
    else:
        nodes = points
        inverse = np.arange(len(points), dtype=np.int64)

    if len(nodes) == 1:
        node_labels = np.zeros(1, dtype=np.int64)
    else:
        tree = cKDTree(nodes)
        pairs = tree.query_pairs(r=tolerance, output_type="ndarray")
        node_labels = connected_components(len(nodes), pairs)

    labels = node_labels[inverse]  # push labels back onto the full-resolution cloud

    order = np.argsort(labels, kind="stable")
    sorted_labels = labels[order]
    boundaries = np.flatnonzero(np.diff(sorted_labels)) + 1
    groups = list(np.split(order, boundaries))

    if range_adaptive and len(groups) > 1:
        # Merge BEFORE filtering, not after. The entire point is that a distant
        # object's ring fragments are individually below min_points — filtering
        # first would discard exactly the pieces that need joining, and the
        # merge would then have nothing left to work with.
        groups = _merge_by_range(
            groups,
            points,
            tolerance=tolerance,
            ring_spacing_rad=ring_spacing_rad,
            range_gain=range_gain,
            sensor_origin=np.asarray(sensor_origin, dtype=np.float64),
            max_tolerance=max_tolerance,
        )

    clusters: List[Cluster] = []
    next_id = 0
    for member_idx in groups:
        if len(member_idx) < min_points or len(member_idx) > max_points:
            continue
        member_pts = points[member_idx]
        bbox_min = member_pts.min(axis=0)
        bbox_max = member_pts.max(axis=0)
        if (bbox_max[2] - bbox_min[2]) < min_height:
            continue
        clusters.append(
            Cluster(
                id=next_id,
                indices=np.sort(member_idx),
                centroid=member_pts.mean(axis=0),
                bbox_min=bbox_min,
                bbox_max=bbox_max,
            )
        )
        next_id += 1

    # Largest first: the zone publisher and the RViz marker array both want the
    # most significant object at a stable index.
    clusters.sort(key=lambda c: c.size, reverse=True)
    for new_id, cluster in enumerate(clusters):
        cluster.id = new_id
    return clusters


def cluster_label_array(points: np.ndarray, clusters: List[Cluster]) -> np.ndarray:
    """Per-point cluster id, with ``-1`` for points in no reported cluster."""
    labels = np.full(len(points), -1, dtype=np.int64)
    for cluster in clusters:
        labels[cluster.indices] = cluster.id
    return labels
