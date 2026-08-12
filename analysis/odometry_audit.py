#!/usr/bin/env python3
"""Compare how far each pose source thinks the robot travelled.

The single most diagnostic number in this stack, and it is not an error metric.
Path length asks "did this source see the motion at all", which localises a
fault to one link in the chain in a way ATE cannot:

    ground truth  10.56 m
    wheel odom    10.88 m   <- healthy, 3% over
    scan odom      ?
    EKF            6.47 m   <- losing 40% of the motion
    AMCL           ?

An EKF shorter than every input it fuses is not drift, it is a fusion fault. An
input that is itself short points at the sensor pipeline instead. The two need
completely different fixes and ATE alone does not tell them apart — a lagging
estimate and a noisy one can report similar ATE.

    python analysis/odometry_audit.py --results results
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

# Order matters: this is the chain, sensor to output.
SOURCES = [
    ("ground_truth", "ground truth (Gazebo)"),
    ("wheel_odom", "wheel odometry"),
    ("scan_odom", "LiDAR scan odometry"),
    ("ekf_odom", "EKF fused output"),
    ("amcl_pose", "AMCL"),
]


def load(path: Path) -> List[Tuple[float, float, float, float]]:
    with open(path, "r", encoding="utf-8", newline="") as handle:
        return [
            (float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw"]))
            for r in csv.DictReader(handle)
        ]


def path_length(rows) -> float:
    return sum(
        math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:])
    )


def audit(results_dir: Path) -> dict:
    out = {}
    for name, label in SOURCES:
        p = results_dir / ("%s.csv" % name)
        if not p.exists():
            out[name] = {"label": label, "present": False}
            continue
        rows = load(p)
        if len(rows) < 2:
            out[name] = {"label": label, "present": True, "samples": len(rows)}
            continue
        out[name] = {
            "label": label,
            "present": True,
            "samples": len(rows),
            "duration_s": rows[-1][0] - rows[0][0],
            "path_length_m": path_length(rows),
            "net_displacement_m": math.hypot(rows[-1][1] - rows[0][1], rows[-1][2] - rows[0][2]),
        }
    return out


def format_report(data: dict) -> str:
    truth = data.get("ground_truth", {})
    reference = truth.get("path_length_m")

    lines = [
        "Odometry audit — path length by source",
        "======================================",
        "",
        "%-24s %9s %9s %9s %8s" % ("source", "samples", "path (m)", "net (m)", "vs truth"),
        "-" * 64,
    ]
    for name, _label in SOURCES:
        d = data.get(name, {})
        if not d.get("present"):
            lines.append("%-24s %9s" % (d.get("label", name), "ABSENT"))
            continue
        if "path_length_m" not in d:
            lines.append("%-24s %9d  (too few samples)" % (d["label"], d.get("samples", 0)))
            continue
        ratio = ""
        if reference and name != "ground_truth":
            ratio = "%+.0f%%" % (100.0 * (d["path_length_m"] / reference - 1.0))
        lines.append(
            "%-24s %9d %9.3f %9.3f %8s"
            % (d["label"], d["samples"], d["path_length_m"], d["net_displacement_m"], ratio)
        )

    lines += ["", "Reading it:"]
    ekf = data.get("ekf_odom", {}).get("path_length_m")
    inputs = [
        data.get(n, {}).get("path_length_m")
        for n in ("wheel_odom", "scan_odom")
    ]
    inputs = [v for v in inputs if v]
    if ekf and inputs and ekf < 0.9 * min(inputs):
        lines.append(
            "  The EKF reports LESS motion than any of its inputs. That is a fusion"
        )
        lines.append(
            "  fault, not drift — check which source is dragging it down and how it is"
        )
        lines.append("  configured (differential vs absolute, and the _config mask).")
    elif ekf and inputs:
        lines.append("  The EKF is consistent with its inputs.")
    if reference:
        wheel = data.get("wheel_odom", {}).get("path_length_m")
        if wheel and wheel > 1.15 * reference:
            lines.append(
                "  Wheel odometry over-reports by %.0f%%, which is wheel slip — exactly"
                % (100.0 * (wheel / reference - 1.0))
            )
            lines.append("  what the LiDAR odometry exists to correct.")
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--results", default="results", help="Directory of pose CSV logs.")
    parser.add_argument("--output", help="Write the audit as JSON here.")
    args = parser.parse_args(argv)

    data = audit(Path(args.results))
    print(format_report(data))

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        print("\nwrote %s" % out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
