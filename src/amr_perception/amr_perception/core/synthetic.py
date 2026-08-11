"""A synthetic, fully labelled warehouse LiDAR.

Why this exists rather than SemanticKITTI: the brief allows either, but
SemanticKITTI is an outdoor automotive dataset under a non-commercial research
licence, and its scene statistics — road surface, parked cars, vegetation —
have almost nothing to do with the geometry this pipeline has to survive. The
things that actually break warehouse segmentation are shelf tops masquerading
as floor, 10 cm racking uprights at 15 m range, and people standing against a
rack face. A ray-traced synthetic scene reproduces exactly those cases, comes
with per-point ground truth for free, and carries no licence encumbrance at all.

The sensor model is a spinning multi-beam unit: ``n_rings`` elevations by
``azimuth_step`` degrees of yaw, which reproduces the property that matters most
for clustering — vertical spacing between rings grows linearly with range, so
distant objects fragment. Uniform random sampling in a box, which is what most
synthetic point-cloud tests use, does not have that property and will make a
clustering tolerance look fine when it is not.

Everything is exact ray/primitive intersection. Range noise and dropout are
applied afterwards so the ground truth labels stay exact.

Frame convention: z = 0 is the floor, matching ``base_footprint``. The sensor
sits at ``sensor_xyz``, on a mast.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# Semantic labels.
GROUND = 0
STRUCTURE = 1  # walls, racking uprights and beams, support columns
PALLET = 2  # pallets and the cartons stacked on them
DYNAMIC = 3  # people and other movers

LABEL_NAMES = {GROUND: "ground", STRUCTURE: "structure", PALLET: "pallet", DYNAMIC: "dynamic"}


@dataclass
class Box:
    """Axis-aligned box primitive."""

    lo: np.ndarray
    hi: np.ndarray
    label: int
    instance: int


@dataclass
class Cylinder:
    """Vertical finite cylinder primitive."""

    centre_xy: np.ndarray
    radius: float
    z_lo: float
    z_hi: float
    label: int
    instance: int


@dataclass
class Scene:
    boxes: List[Box] = field(default_factory=list)
    cylinders: List[Cylinder] = field(default_factory=list)
    floor_z: float = 0.0
    _next_instance: int = 1

    def add_box(self, lo, hi, label: int) -> int:
        inst = self._next_instance
        self._next_instance += 1
        self.boxes.append(
            Box(
                lo=np.asarray(lo, dtype=np.float64),
                hi=np.asarray(hi, dtype=np.float64),
                label=label,
                instance=inst,
            )
        )
        return inst

    def add_cylinder(self, centre_xy, radius: float, z_lo: float, z_hi: float, label: int) -> int:
        inst = self._next_instance
        self._next_instance += 1
        self.cylinders.append(
            Cylinder(
                centre_xy=np.asarray(centre_xy, dtype=np.float64),
                radius=float(radius),
                z_lo=float(z_lo),
                z_hi=float(z_hi),
                label=label,
                instance=inst,
            )
        )
        return inst

    @property
    def instance_labels(self) -> dict:
        out = {0: GROUND}
        for box in self.boxes:
            out[box.instance] = box.label
        for cyl in self.cylinders:
            out[cyl.instance] = cyl.label
        return out

    def primitive_bounds(self) -> Dict[int, Tuple[np.ndarray, np.ndarray]]:
        """Axis-aligned bounds per primitive instance.

        A cylinder is bounded by its enclosing box, which overestimates its
        extent and therefore *underestimates* the gap to its neighbours. That
        errs towards declaring two primitives connected, which is the safe
        direction for the grouping below: it never claims two objects are
        separable when they are not.
        """
        out: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        for box in self.boxes:
            out[box.instance] = (box.lo.copy(), box.hi.copy())
        for cyl in self.cylinders:
            r = cyl.radius
            out[cyl.instance] = (
                np.array([cyl.centre_xy[0] - r, cyl.centre_xy[1] - r, cyl.z_lo]),
                np.array([cyl.centre_xy[0] + r, cyl.centre_xy[1] + r, cyl.z_hi]),
            )
        return out

    def object_groups(self, tolerance: float = 0.35) -> Dict[int, int]:
        """Group primitives into physically connected objects.

        Returns a mapping from primitive instance id to an object id.

        This is the ground truth a Euclidean clusterer should be scored
        against, and scoring against raw primitives instead is a mistake worth
        spelling out. A racking bay is modelled here as separate uprights,
        beams, pallets and cartons, but they are all in contact: no
        connectivity-based clustering can separate them, and none should be
        expected to. Treating each primitive as a target instance measures
        nothing except how finely the scene happens to be modelled.

        Two primitives join the same object when the gap between their bounds is
        below ``tolerance`` — the same radius the clustering uses, so the
        grouping asks exactly the question the algorithm answers.
        """
        bounds = self.primitive_bounds()
        ids = sorted(bounds)
        parent = {i: i for i in ids}

        def find(a: int) -> int:
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        for index, a in enumerate(ids):
            a_lo, a_hi = bounds[a]
            for b in ids[index + 1 :]:
                b_lo, b_hi = bounds[b]
                gap = np.maximum(0.0, np.maximum(b_lo - a_hi, a_lo - b_hi))
                if float(np.linalg.norm(gap)) < tolerance:
                    union(a, b)

        roots = {}
        out: Dict[int, int] = {}
        for i in ids:
            root = find(i)
            if root not in roots:
                roots[root] = len(roots) + 1  # object ids start at 1; 0 is the floor
            out[i] = roots[root]
        return out


@dataclass
class Scan:
    """One simulated revolution."""

    points: np.ndarray  # (N, 3) in the floor frame
    labels: np.ndarray  # (N,) semantic label
    instances: np.ndarray  # (N,) instance id, 0 for the floor
    sensor_xyz: np.ndarray

    def __len__(self) -> int:
        return len(self.points)

    @property
    def ground_mask(self) -> np.ndarray:
        return self.labels == GROUND

    def object_instances(self, min_points: int = 1) -> dict:
        """Instance id -> point count, floor excluded."""
        out = {}
        for inst, count in zip(*np.unique(self.instances, return_counts=True)):
            if inst == 0:
                continue
            if count >= min_points:
                out[int(inst)] = int(count)
        return out


def ray_directions(
    n_rings: int = 16,
    elevation_range_deg: Tuple[float, float] = (-15.0, 15.0),
    azimuth_step_deg: float = 0.2,
) -> np.ndarray:
    """Unit direction for every beam of one revolution, shape (n_rings * cols, 3)."""
    elev = np.deg2rad(np.linspace(elevation_range_deg[0], elevation_range_deg[1], n_rings))
    n_cols = int(round(360.0 / azimuth_step_deg))
    azim = np.deg2rad(np.arange(n_cols) * azimuth_step_deg)

    el = np.repeat(elev, n_cols)
    az = np.tile(azim, n_rings)

    cos_el = np.cos(el)
    return np.stack([cos_el * np.cos(az), cos_el * np.sin(az), np.sin(el)], axis=1)


def _intersect_plane(origin: np.ndarray, dirs: np.ndarray, z: float) -> np.ndarray:
    """Ray parameter t for a horizontal plane, inf where there is no hit."""
    dz = dirs[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (z - origin[2]) / dz
    t = np.where((np.abs(dz) > 1e-12) & (t > 0), t, np.inf)
    return t


def _intersect_box(origin: np.ndarray, dirs: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Slab method. Returns entry distance, inf where the ray misses."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / dirs
        t0 = (lo - origin) * inv
        t1 = (hi - origin) * inv
    # A zero direction component gives +-inf here, which the min/max below
    # handle correctly: the ray is parallel to that slab and either always
    # inside it or always outside.
    t_near = np.nanmax(np.minimum(t0, t1), axis=1)
    t_far = np.nanmin(np.maximum(t0, t1), axis=1)

    hit = (t_far >= np.maximum(t_near, 0.0)) & (t_far > 0)
    t = np.where(t_near > 0, t_near, t_far)
    return np.where(hit & (t > 0), t, np.inf)


def _intersect_cylinder(origin: np.ndarray, dirs: np.ndarray, cyl: Cylinder) -> np.ndarray:
    """Nearest hit on a vertical finite cylinder's side surface."""
    ox = origin[0] - cyl.centre_xy[0]
    oy = origin[1] - cyl.centre_xy[1]
    dx = dirs[:, 0]
    dy = dirs[:, 1]

    a = dx * dx + dy * dy
    b = 2.0 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - cyl.radius * cyl.radius
    disc = b * b - 4.0 * a * c

    valid = (a > 1e-12) & (disc >= 0)
    sqrt_disc = np.sqrt(np.where(valid, disc, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        t_near = (-b - sqrt_disc) / (2.0 * a)
        t_far = (-b + sqrt_disc) / (2.0 * a)

    result = np.full(len(dirs), np.inf)
    for t in (t_near, t_far):
        z = origin[2] + t * dirs[:, 2]
        ok = valid & (t > 0) & (z >= cyl.z_lo) & (z <= cyl.z_hi) & (t < result)
        result = np.where(ok, t, result)
    return result


def simulate_scan(
    scene: Scene,
    sensor_xyz=(0.0, 0.0, 0.9),
    n_rings: int = 16,
    elevation_range_deg: Tuple[float, float] = (-15.0, 15.0),
    azimuth_step_deg: float = 0.2,
    max_range: float = 40.0,
    min_range: float = 0.6,
    range_sigma: float = 0.01,
    dropout: float = 0.01,
    seed: int = 0,
) -> Scan:
    """Trace one revolution of the sensor through ``scene``.

    ``range_sigma`` (1 cm) and ``dropout`` (1%) match the ``gaussianNoise`` and
    typical return rate configured on the Gazebo ray sensor in
    ``amr_description``, so metrics measured here transfer to the simulated run.
    """
    origin = np.asarray(sensor_xyz, dtype=np.float64).reshape(3)
    dirs = ray_directions(n_rings, elevation_range_deg, azimuth_step_deg)
    n = len(dirs)

    best_t = np.full(n, np.inf)
    best_label = np.zeros(n, dtype=np.int64)
    best_instance = np.zeros(n, dtype=np.int64)

    t_floor = _intersect_plane(origin, dirs, scene.floor_z)
    closer = t_floor < best_t
    best_t = np.where(closer, t_floor, best_t)
    best_label = np.where(closer, GROUND, best_label)
    best_instance = np.where(closer, 0, best_instance)

    for box in scene.boxes:
        t = _intersect_box(origin, dirs, box.lo, box.hi)
        closer = t < best_t
        best_t = np.where(closer, t, best_t)
        best_label = np.where(closer, box.label, best_label)
        best_instance = np.where(closer, box.instance, best_instance)

    for cyl in scene.cylinders:
        t = _intersect_cylinder(origin, dirs, cyl)
        closer = t < best_t
        best_t = np.where(closer, t, best_t)
        best_label = np.where(closer, cyl.label, best_label)
        best_instance = np.where(closer, cyl.instance, best_instance)

    keep = np.isfinite(best_t) & (best_t >= min_range) & (best_t <= max_range)

    rng = np.random.default_rng(seed)
    if dropout > 0:
        keep &= rng.random(n) >= dropout

    t = best_t[keep]
    d = dirs[keep]
    if range_sigma > 0:
        t = t + rng.normal(0.0, range_sigma, size=len(t))

    points = origin[None, :] + t[:, None] * d

    return Scan(
        points=points,
        labels=best_label[keep],
        instances=best_instance[keep],
        sensor_xyz=origin,
    )


def warehouse_scene(
    length: float = 30.0,
    width: float = 12.0,
    aisle_half_width: float = 1.4,
    n_bays: int = 10,
    people: Optional[List[Tuple[float, float]]] = None,
    floor_pallets: Optional[List[Tuple[float, float]]] = None,
) -> Scene:
    """The aisle the demo drives down: two racking runs, pallets, people.

    Geometry matches ``simulation/worlds/warehouse.world`` so that metrics
    measured against this scene describe the same problem the Gazebo run poses.
    """
    scene = Scene()
    half_l = length / 2.0
    half_w = width / 2.0

    # Perimeter walls, 0.2 m thick, 6 m to the roof steel.
    scene.add_box([-half_l - 0.2, -half_w - 0.2, 0.0], [-half_l, half_w + 0.2, 6.0], STRUCTURE)
    scene.add_box([half_l, -half_w - 0.2, 0.0], [half_l + 0.2, half_w + 0.2, 6.0], STRUCTURE)
    scene.add_box([-half_l - 0.2, -half_w - 0.2, 0.0], [half_l + 0.2, -half_w, 6.0], STRUCTURE)
    scene.add_box([-half_l - 0.2, half_w, 0.0], [half_l + 0.2, half_w + 0.2, 6.0], STRUCTURE)

    # Two racking runs flanking the aisle. Uprights every 2.7 m, paired 0.9 m
    # apart across the bay depth, plus beams at 0.8 m and 1.6 m.
    bay_pitch = 2.7
    rack_depth = 0.9
    for side in (-1.0, 1.0):
        y_front = side * aisle_half_width
        y_back = side * (aisle_half_width + rack_depth)
        y_lo, y_hi = min(y_front, y_back), max(y_front, y_back)

        for bay in range(n_bays + 1):
            x = -half_l + 1.5 + bay * bay_pitch
            if x > half_l - 1.0:
                break
            for y in (y_front, y_back):
                scene.add_box([x - 0.05, y - 0.05, 0.0], [x + 0.05, y + 0.05, 2.2], STRUCTURE)

        for beam_z in (0.80, 1.60):
            scene.add_box(
                [-half_l + 1.45, y_lo, beam_z],
                [-half_l + 1.5 + n_bays * bay_pitch, y_hi, beam_z + 0.08],
                STRUCTURE,
            )

        # A pallet with a carton stack in most bays, but not all: gaps in the
        # racking are what let the sensor see the far wall, and a scene with no
        # gaps hides the shelf-top-as-floor failure mode entirely.
        for bay in range(n_bays):
            if bay % 3 == 2:
                continue
            x = -half_l + 1.5 + bay * bay_pitch + bay_pitch / 2.0
            if x > half_l - 1.0:
                break
            for level_z in (0.88, 1.68):
                scene.add_box([x - 0.6, y_lo + 0.05, level_z], [x + 0.6, y_hi - 0.05, level_z + 0.15], PALLET)
                scene.add_box(
                    [x - 0.5, y_lo + 0.10, level_z + 0.15],
                    [x + 0.5, y_hi - 0.10, level_z + 0.75],
                    PALLET,
                )

    # Roof support columns, which are the other thing that looks like a person
    # to a naive clusterer.
    for x in (-half_l / 2.0, half_l / 2.0):
        scene.add_cylinder([x, half_w - 0.8], 0.15, 0.0, 6.0, STRUCTURE)

    # Pallets left in the aisle: the obstacles Nav2 has to plan around.
    if floor_pallets is None:
        floor_pallets = [(4.0, 0.35), (-6.5, -0.4)]
    for px, py in floor_pallets:
        scene.add_box([px - 0.6, py - 0.4, 0.0], [px + 0.6, py + 0.4, 0.15], PALLET)
        scene.add_box([px - 0.5, py - 0.3, 0.15], [px + 0.5, py + 0.3, 0.95], PALLET)

    # People. 0.25 m radius, 1.75 m tall.
    if people is None:
        people = [(8.0, 0.0), (-3.0, 1.0), (14.0, -0.8)]
    for px, py in people:
        scene.add_cylinder([px, py], 0.25, 0.0, 1.75, DYNAMIC)

    return scene
