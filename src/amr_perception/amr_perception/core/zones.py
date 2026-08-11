"""Work-zone occupancy from clustered LiDAR returns.

A zone is a vertical prism: an arbitrary polygon on the floor plane plus a
height band. The question the publisher answers is "is there something in this
zone right now", which downstream is used to hold the AMR out of a pick face
while a human is working it.

Two design points are worth stating because they are the difference between a
demo and something you would actually let gate a robot:

**Occupancy is decided on raw point count inside the prism, not on cluster
centroids.** A person standing at the very edge of a zone has a centroid outside
it, and centroid-based occupancy declares that zone clear. Counting the returns
inside the prism does not have that failure mode.

**State is debounced with asymmetric hysteresis.** A single dropped frame, or a
person momentarily occluded by racking, must not flip a zone to clear. Entering
the occupied state takes fewer consecutive frames than leaving it, so the
publisher is quick to notice a person and slow to forget them. That asymmetry is
deliberate and the numbers are exposed as parameters.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class Zone:
    """A named vertical prism over a floor polygon."""

    name: str
    polygon: np.ndarray  # (M, 2) in the map frame, ordered, implicitly closed
    z_min: float = 0.05
    z_max: float = 2.20
    min_points: int = 15
    description: str = ""

    def __post_init__(self) -> None:
        poly = np.asarray(self.polygon, dtype=np.float64)
        if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3:
            raise ValueError(
                "zone %r polygon must have shape (>=3, 2), got %r"
                % (self.name, poly.shape)
            )
        if self.z_max <= self.z_min:
            raise ValueError("zone %r has z_max <= z_min" % (self.name,))
        object.__setattr__(self, "polygon", poly)

    @property
    def area(self) -> float:
        """Polygon area by the shoelace formula."""
        x = self.polygon[:, 0]
        y = self.polygon[:, 1]
        return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))

    @property
    def centroid(self) -> np.ndarray:
        return self.polygon.mean(axis=0)


@dataclass
class ZoneState:
    """Instantaneous, undebounced measurement for one zone."""

    name: str
    occupied: bool
    point_count: int
    cluster_ids: List[int] = field(default_factory=list)
    nearest_cluster_distance: float = float("inf")

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "occupied": bool(self.occupied),
            "point_count": int(self.point_count),
            "cluster_ids": [int(i) for i in self.cluster_ids],
            "nearest_cluster_distance": (
                None
                if not np.isfinite(self.nearest_cluster_distance)
                else float(self.nearest_cluster_distance)
            ),
        }


def points_in_polygon(points_xy: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Vectorised even-odd (ray casting) point-in-polygon test.

    Handles concave polygons. Points exactly on an edge are not guaranteed to
    land on either side consistently, which is accepted: zone boundaries are
    drawn with a margin precisely so that sub-centimetre edge cases cannot
    matter.
    """
    points_xy = np.asarray(points_xy, dtype=np.float64)
    if len(points_xy) == 0:
        return np.zeros(0, dtype=bool)

    poly = np.asarray(polygon, dtype=np.float64)
    x = points_xy[:, 0][:, None]
    y = points_xy[:, 1][:, None]

    x1 = poly[:, 0][None, :]
    y1 = poly[:, 1][None, :]
    x2 = np.roll(poly[:, 0], -1)[None, :]
    y2 = np.roll(poly[:, 1], -1)[None, :]

    # Edge straddles the horizontal ray cast from the test point.
    straddles = (y1 > y) != (y2 > y)
    # x coordinate of the intersection, guarding the horizontal-edge divide.
    denom = np.where(np.abs(y2 - y1) < 1e-18, 1e-18, y2 - y1)
    x_int = x1 + (y - y1) * (x2 - x1) / denom
    crossings = straddles & (x < x_int)
    return (crossings.sum(axis=1) % 2) == 1


def evaluate_zone(
    zone: Zone,
    points: np.ndarray,
    clusters: Optional[Sequence] = None,
) -> ZoneState:
    """Measure one zone against a cloud of non-ground points in the map frame."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) == 0:
        return ZoneState(name=zone.name, occupied=False, point_count=0)

    in_band = (points[:, 2] >= zone.z_min) & (points[:, 2] <= zone.z_max)
    inside = np.zeros(len(points), dtype=bool)
    if in_band.any():
        band_idx = np.flatnonzero(in_band)
        inside[band_idx] = points_in_polygon(points[band_idx, :2], zone.polygon)

    count = int(inside.sum())

    cluster_ids: List[int] = []
    nearest = float("inf")
    if clusters:
        inside_idx = set(np.flatnonzero(inside).tolist())
        for cluster in clusters:
            if inside_idx.intersection(cluster.indices.tolist()):
                cluster_ids.append(int(cluster.id))
                nearest = 0.0
            else:
                dist = float(np.linalg.norm(cluster.centroid[:2] - zone.centroid))
                nearest = min(nearest, dist)

    return ZoneState(
        name=zone.name,
        occupied=count >= zone.min_points,
        point_count=count,
        cluster_ids=cluster_ids,
        nearest_cluster_distance=nearest,
    )


def evaluate_zones(
    zones: Iterable[Zone],
    points: np.ndarray,
    clusters: Optional[Sequence] = None,
) -> List[ZoneState]:
    return [evaluate_zone(zone, points, clusters) for zone in zones]


class ZoneOccupancyTracker:
    """Debounces raw per-frame zone measurements into a published state.

    ``enter_frames`` consecutive occupied measurements are needed to latch
    occupied; ``exit_frames`` consecutive clear measurements to release it. The
    defaults (2 in, 5 out) come from the 10 Hz publish rate: a zone latches
    within 200 ms of someone stepping into it, and stays latched for half a
    second after the last return, which covers the worst observed occlusion gap
    when a person passes behind a racking upright.
    """

    def __init__(self, enter_frames: int = 2, exit_frames: int = 5) -> None:
        if enter_frames < 1 or exit_frames < 1:
            raise ValueError("enter_frames and exit_frames must both be >= 1")
        self.enter_frames = enter_frames
        self.exit_frames = exit_frames
        self._latched: Dict[str, bool] = {}
        self._streak: Dict[str, int] = {}

    def update(self, states: Iterable[ZoneState]) -> Dict[str, bool]:
        """Feed one frame of measurements, get the debounced states back."""
        for state in states:
            name = state.name
            latched = self._latched.get(name, False)
            if state.occupied == latched:
                self._streak[name] = 0
                continue

            self._streak[name] = self._streak.get(name, 0) + 1
            needed = self.enter_frames if state.occupied else self.exit_frames
            if self._streak[name] >= needed:
                self._latched[name] = state.occupied
                self._streak[name] = 0
        return dict(self._latched)

    @property
    def latched(self) -> Dict[str, bool]:
        return dict(self._latched)

    def reset(self) -> None:
        self._latched.clear()
        self._streak.clear()


def load_zones(path: str) -> List[Zone]:
    """Load zone definitions from the YAML format used in ``simulation/zones``."""
    import yaml  # imported lazily so the core stays importable without PyYAML

    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    if not isinstance(raw, dict) or "zones" not in raw:
        raise ValueError("%s: expected a top-level 'zones' key" % path)

    zones: List[Zone] = []
    for entry in raw["zones"]:
        zones.append(
            Zone(
                name=entry["name"],
                polygon=np.asarray(entry["polygon"], dtype=np.float64),
                z_min=float(entry.get("z_min", 0.05)),
                z_max=float(entry.get("z_max", 2.20)),
                min_points=int(entry.get("min_points", 15)),
                description=str(entry.get("description", "")),
            )
        )
    if not zones:
        raise ValueError("%s: no zones defined" % path)
    return zones
