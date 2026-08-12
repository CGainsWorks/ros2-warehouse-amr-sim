#!/usr/bin/env python3
"""Segmentation accuracy and per-scan latency.

Runs the real ground-removal and clustering code over ray-traced scans of the
warehouse scene, which carry exact per-point semantic and instance labels, and
reports what the pipeline recovered.

This is the script behind every segmentation number quoted in the README, and it
needs nothing but numpy and scipy — no ROS, no Gazebo, no dataset download::

    python analysis/segmentation_metrics.py --output results/segmentation.json

WHAT IS MEASURED, AND WHY IT IS MEASURED THIS WAY

*Ground removal* is scored per point as a binary classification. Precision
matters more than recall here and the two are not symmetric in consequence: a
ground point wrongly kept becomes a phantom obstacle in the costmap and can stop
the robot dead, whereas an object point wrongly discarded as ground is only
dangerous if the whole object goes, which the clustering stage would then miss
entirely. Both are reported.

*Clustering* is scored per object rather than per point, because per-point
clustering accuracy is close to meaningless — it rewards getting the big flat
surfaces right, which is easy, and says nothing about whether the 0.25 m person
at 12 m was found.

The definition of "object" here matters, and getting it wrong makes the whole
metric worthless. The scene models a racking bay as separate uprights, beams,
pallets and cartons, but those are all in physical contact. No connectivity-based
clustering can separate them and none should be expected to; scoring against
individual primitives measures how finely the scene was modelled and nothing
else. Ground truth is therefore ``Scene.object_groups()`` — primitives grouped
into connected components under the same radius the clustering uses. A racking
run is one object, and one cluster covering it is a correct answer.

An object counts as recovered when some single cluster covers at least
``--match-iou`` of its points by IoU. Over-segmentation (one object split across
clusters) and under-segmentation (several objects merged into one) are counted
separately, since they have very different downstream consequences: a split
object still gets avoided, a merged one can hide a person against a rack face.

*Latency* is wall-clock per stage, reported as percentiles. The mean is not
reported: the mean hides the tail, and the tail is what causes a dropped scan.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
for _pkg in ("amr_common", "amr_perception"):
    _path = str(REPO_ROOT / "src" / _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from amr_perception.core.clustering import cluster_points  # noqa: E402
from amr_perception.core.ground_removal import remove_ground  # noqa: E402
from amr_perception.core.occupancy_3d import build_occupancy  # noqa: E402
from amr_perception.core.synthetic import (  # noqa: E402
    GROUND,
    LABEL_NAMES,
    simulate_scan,
    warehouse_scene,
)


@dataclass
class ScanMetrics:
    ground_tp: int = 0
    ground_fp: int = 0
    ground_fn: int = 0
    ground_tn: int = 0

    instances_present: int = 0
    instances_recovered: int = 0
    instances_over_segmented: int = 0
    instances_under_segmented: int = 0
    clusters_reported: int = 0
    clusters_spurious: int = 0

    ground_ms: float = 0.0
    cluster_ms: float = 0.0
    occupancy_ms: float = 0.0

    per_label_present: Dict[int, int] = field(default_factory=dict)
    per_label_recovered: Dict[int, int] = field(default_factory=dict)

    @property
    def total_ms(self) -> float:
        return self.ground_ms + self.cluster_ms + self.occupancy_ms


def _safe_div(numerator: float, denominator: float) -> float:
    return float(numerator / denominator) if denominator else 0.0


def evaluate_scan(
    scan,
    object_groups: Dict[int, int],
    min_instance_points: int = 25,
    match_iou: float = 0.5,
    ground_threshold: float = 0.06,
    cluster_tolerance: float = 0.35,
    cluster_min_points: int = 25,
    cluster_voxel: float = 0.12,
) -> ScanMetrics:
    """Score one labelled scan through the real pipeline.

    ``object_groups`` maps primitive instance id to connected-object id; see
    ``Scene.object_groups``.
    """
    metrics = ScanMetrics()
    points = scan.points

    started = time.perf_counter()
    ground_mask, object_mask, _plane = remove_ground(
        points, distance_threshold=ground_threshold, max_height=3.0
    )
    metrics.ground_ms = (time.perf_counter() - started) * 1e3

    truth_ground = scan.labels == GROUND
    metrics.ground_tp = int((ground_mask & truth_ground).sum())
    metrics.ground_fp = int((ground_mask & ~truth_ground).sum())
    metrics.ground_fn = int((~ground_mask & truth_ground).sum())
    metrics.ground_tn = int((~ground_mask & ~truth_ground).sum())

    object_points = points[object_mask]
    object_labels = scan.labels[object_mask]
    # Collapse primitive instances onto connected-object ids. Anything not in
    # the grouping (only the floor, id 0) stays 0 and is excluded below.
    object_instances = np.array(
        [object_groups.get(int(i), 0) for i in scan.instances[object_mask]], dtype=np.int64
    )

    started = time.perf_counter()
    clusters = cluster_points(
        object_points,
        tolerance=cluster_tolerance,
        min_points=cluster_min_points,
        voxel_size=cluster_voxel,
        min_height=0.10,
        # The synthetic scan is expressed in the floor frame, so the sensor is
        # not at the origin. The range-adaptive tolerance is measured from the
        # sensor, and getting this wrong understates range and therefore
        # under-merges exactly the distant objects the adaptation is for.
        sensor_origin=tuple(scan.sensor_xyz),
    )
    metrics.cluster_ms = (time.perf_counter() - started) * 1e3
    metrics.clusters_reported = len(clusters)

    started = time.perf_counter()
    build_occupancy(object_points, resolution=0.10)
    metrics.occupancy_ms = (time.perf_counter() - started) * 1e3

    # --- object recovery ---------------------------------------------------
    #
    # Only objects that survived ground removal with enough points to be
    # findable are counted as "present". Scoring the pipeline against something
    # that is two returns at 30 m measures the sensor, not the algorithm.
    present: Dict[int, np.ndarray] = {}
    for inst in np.unique(object_instances):
        if inst == 0:
            continue
        member = object_instances == inst
        if int(member.sum()) >= min_instance_points:
            present[int(inst)] = member

    metrics.instances_present = len(present)

    cluster_masks = []
    for cluster in clusters:
        mask = np.zeros(len(object_points), dtype=bool)
        mask[cluster.indices] = True
        cluster_masks.append(mask)

    matched_clusters = set()
    for inst, member in present.items():
        # A grouped object can span labels — a racking run is structure plus the
        # pallets stacked on it — so attribute it to its majority label rather
        # than to whichever point happens to come first.
        counts = np.bincount(object_labels[member])
        label = int(np.argmax(counts))
        metrics.per_label_present[label] = metrics.per_label_present.get(label, 0) + 1

        overlaps = []
        for index, mask in enumerate(cluster_masks):
            intersection = int((member & mask).sum())
            if intersection == 0:
                continue
            union = int((member | mask).sum())
            overlaps.append((index, intersection / union, intersection / int(member.sum())))

        if not overlaps:
            continue

        best = max(overlaps, key=lambda item: item[1])
        # Recovered if some single cluster covers at least match_iou of the
        # instance's points. IoU rather than recall alone, so a cluster that
        # swallows the object plus half the racking does not count as a
        # successful detection.
        # Every cluster overlapping a known object is accounted for, whether or
        # not it was the one that won the match. A racking run that fragments at
        # range produces one cluster that clears the IoU threshold and several
        # that do not, and those others are fragments of a known object, not
        # spurious detections. Counting them as spurious would report a false
        # alarm rate that is really an over-segmentation rate, which is already
        # measured separately.
        matched_clusters.update(index for index, _, _ in overlaps)

        if best[1] >= match_iou:
            metrics.instances_recovered += 1
            metrics.per_label_recovered[label] = metrics.per_label_recovered.get(label, 0) + 1
        elif best[2] >= match_iou:
            # Most of the object is in one cluster, but that cluster is much
            # bigger than the object: it has been merged with something else.
            metrics.instances_under_segmented += 1
        else:
            # No single cluster holds enough of it: the object has been split.
            metrics.instances_over_segmented += 1

    # Spurious means "overlaps no labelled object at all" — a detection out of
    # nothing, which is the thing that would stop the robot for no reason.
    metrics.clusters_spurious = len(cluster_masks) - len(matched_clusters)
    return metrics


def aggregate(runs: Sequence[ScanMetrics]) -> dict:
    """Pool per-scan metrics into the summary that gets reported."""
    tp = sum(r.ground_tp for r in runs)
    fp = sum(r.ground_fp for r in runs)
    fn = sum(r.ground_fn for r in runs)
    tn = sum(r.ground_tn for r in runs)

    precision = _safe_div(tp, tp + fp)
    recall = _safe_div(tp, tp + fn)

    ground_ms = np.array([r.ground_ms for r in runs])
    cluster_ms = np.array([r.cluster_ms for r in runs])
    occupancy_ms = np.array([r.occupancy_ms for r in runs])
    total_ms = np.array([r.total_ms for r in runs])

    present = sum(r.instances_present for r in runs)
    recovered = sum(r.instances_recovered for r in runs)

    per_label: Dict[str, dict] = {}
    for label, name in LABEL_NAMES.items():
        if label == GROUND:
            continue
        p = sum(r.per_label_present.get(label, 0) for r in runs)
        rec = sum(r.per_label_recovered.get(label, 0) for r in runs)
        if p:
            per_label[name] = {"present": p, "recovered": rec, "rate": _safe_div(rec, p)}

    return {
        "scans": len(runs),
        "ground_removal": {
            "precision": precision,
            "recall": recall,
            "f1": _safe_div(2 * precision * recall, precision + recall),
            "iou": _safe_div(tp, tp + fp + fn),
            "accuracy": _safe_div(tp + tn, tp + tn + fp + fn),
            "true_positives": tp,
            "false_positives": fp,
            "false_negatives": fn,
        },
        "clustering": {
            "instances_present": present,
            "instances_recovered": recovered,
            "recovery_rate": _safe_div(recovered, present),
            "under_segmented": sum(r.instances_under_segmented for r in runs),
            "over_segmented": sum(r.instances_over_segmented for r in runs),
            "clusters_reported": sum(r.clusters_reported for r in runs),
            "clusters_spurious": sum(r.clusters_spurious for r in runs),
            "per_label": per_label,
        },
        "latency_ms": {
            "ground_removal_median": float(np.median(ground_ms)),
            "ground_removal_p95": float(np.percentile(ground_ms, 95)),
            "clustering_median": float(np.median(cluster_ms)),
            "clustering_p95": float(np.percentile(cluster_ms, 95)),
            "occupancy_median": float(np.median(occupancy_ms)),
            "occupancy_p95": float(np.percentile(occupancy_ms, 95)),
            "total_median": float(np.median(total_ms)),
            "total_p95": float(np.percentile(total_ms, 95)),
            "total_max": float(total_ms.max()),
            "budget_ms": 100.0,
            "scans_over_budget": int((total_ms > 100.0).sum()),
        },
    }


def sensor_poses(n: int, x_min: float = -12.0, x_max: float = 12.0) -> List[Tuple[float, float, float]]:
    """Sensor positions along the aisle centreline.

    Spread along the full aisle rather than clustered at one spot: range is the
    dominant driver of both point density and clustering difficulty, so a
    benchmark taken from a single position characterises one range and nothing
    else.
    """
    xs = np.linspace(x_min, x_max, n)
    return [(float(x), 0.0, 0.9) for x in xs]


def run_benchmark(
    n_scans: int = 25,
    seed: int = 0,
    match_iou: float = 0.5,
    cluster_tolerance: float = 0.35,
    **kwargs,
) -> dict:
    scene = warehouse_scene()
    # Ground truth is grouped at the same radius the clustering uses, so the
    # metric asks exactly the question the algorithm answers.
    groups = scene.object_groups(tolerance=cluster_tolerance)

    runs = []
    for index, pose in enumerate(sensor_poses(n_scans)):
        scan = simulate_scan(scene, sensor_xyz=pose, seed=seed + index)
        runs.append(
            evaluate_scan(
                scan,
                groups,
                match_iou=match_iou,
                cluster_tolerance=cluster_tolerance,
                **kwargs,
            )
        )
    result = aggregate(runs)
    result["config"] = {
        "n_scans": n_scans,
        "seed": seed,
        "match_iou": match_iou,
        "cluster_tolerance": cluster_tolerance,
        "scene": "amr_perception.core.synthetic.warehouse_scene",
        "distinct_objects_in_scene": len(set(groups.values())),
        **{k: v for k, v in kwargs.items()},
    }
    return result


def format_report(result: dict) -> str:
    ground = result["ground_removal"]
    clustering = result["clustering"]
    latency = result["latency_ms"]

    lines = [
        "Segmentation metrics",
        "====================",
        "%d scans of the synthetic warehouse scene" % result["scans"],
        "",
        "Ground removal (per point)",
        "  precision        %.4f" % ground["precision"],
        "  recall           %.4f" % ground["recall"],
        "  F1               %.4f" % ground["f1"],
        "  IoU              %.4f" % ground["iou"],
        "  false positives  %d ground points kept as obstacles" % ground["false_positives"],
        "  false negatives  %d object points discarded as ground" % ground["false_negatives"],
        "",
        "Clustering (per instance)",
        "  present          %d" % clustering["instances_present"],
        "  recovered        %d (%.1f%%)"
        % (clustering["instances_recovered"], 100.0 * clustering["recovery_rate"]),
        "  under-segmented  %d (merged into a larger cluster)" % clustering["under_segmented"],
        "  over-segmented   %d (split across clusters)" % clustering["over_segmented"],
        "  spurious         %d clusters matching no labelled instance" % clustering["clusters_spurious"],
    ]
    for name, stats in sorted(clustering["per_label"].items()):
        lines.append(
            "    %-10s   %d/%d (%.1f%%)"
            % (name, stats["recovered"], stats["present"], 100.0 * stats["rate"])
        )

    lines += [
        "",
        "Latency per scan (ms)",
        "  ground removal   median %.2f   p95 %.2f"
        % (latency["ground_removal_median"], latency["ground_removal_p95"]),
        "  clustering       median %.2f   p95 %.2f"
        % (latency["clustering_median"], latency["clustering_p95"]),
        "  occupancy grid   median %.2f   p95 %.2f"
        % (latency["occupancy_median"], latency["occupancy_p95"]),
        "  TOTAL            median %.2f   p95 %.2f   max %.2f"
        % (latency["total_median"], latency["total_p95"], latency["total_max"]),
        "  budget           %.0f ms at 10 Hz; %d scans over budget"
        % (latency["budget_ms"], latency["scans_over_budget"]),
    ]
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scans", type=int, default=25, help="Sensor positions along the aisle.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--match-iou", type=float, default=0.5)
    parser.add_argument("--output", help="Write the result as JSON here.")
    args = parser.parse_args(argv)

    result = run_benchmark(n_scans=args.scans, seed=args.seed, match_iou=args.match_iou)
    print(format_report(result))

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print("\nwrote %s" % out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
