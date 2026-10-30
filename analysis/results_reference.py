#!/usr/bin/env python3
"""The committed reference for the simulated localisation numbers the README quotes.

``results/`` is generated output and is not tracked, with one exception:
``results/reference/gazebo_localisation.json``, a compact summary of one
recorded open-loop run and one recorded mission on gz-sim. The README's
simulated tables are rendered from it, and ``tests/test_results_reference.py``
fails CI if a table in the README stops matching the file.

    # after `make openloop` and `make eval`:
    python3 analysis/results_reference.py build \\
        --openloop results/openloop --mission results \\
        --output results/reference/gazebo_localisation.json
    python3 analysis/results_reference.py render   # the README tables

Simulated runs are not bit reproducible across machines, so CI does not rerun
these and compare; it checks that the README quotes the recorded file
faithfully. The integration tests in CI rerun a shorter mission every push and
assert bounds on it instead.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
REFERENCE = REPO_ROOT / "results" / "reference" / "gazebo_localisation.json"
README = REPO_ROOT / "README.md"

# Source key -> (label in the README, error JSON written by the run scripts,
# pose log the odometry audit names it by).
SOURCES = (
    ("wheel_odometry", "Wheel odometry", "wheel_odom_error.json", "wheel_odom"),
    ("ekf", "EKF fused", "ekf_error.json", "ekf_odom"),
    ("amcl", "AMCL", "localisation_error.json", "amcl_pose"),
)
RUNS = ("openloop", "mission")


def _source_summary(error: dict, audit: dict, audit_key: str, truth_path: float) -> dict:
    path = audit.get(audit_key, {}).get("path_length_m")
    nees = error.get("nees_mean")
    return {
        "ate_rmse_m": round(float(error["ate_rmse_m"]), 3),
        "ate_max_m": round(float(error["ate_max_m"]), 3),
        "yaw_rmse_deg": round(float(error["yaw_rmse_deg"]), 2),
        "nees_mean": None if nees is None else round(float(nees), 2),
        "jumps_over_0p5m": int(error["n_jumps_over_0p5m"]),
        "path_vs_truth_pct": None if not path or not truth_path
        else round(100.0 * (float(path) - truth_path) / truth_path, 1) + 0.0,  # + 0.0 drops a -0.0
    }


def summarise_run(run_dir: Path) -> dict:
    audit = json.loads((run_dir / "odometry_audit.json").read_text(encoding="utf-8"))
    truth_path = float(audit["ground_truth"]["path_length_m"])
    out = {
        "duration_s": round(float(audit["ground_truth"]["duration_s"]), 1),
        "ground_truth_path_m": round(truth_path, 2),
        "sources": {},
    }
    for key, _label, error_file, audit_key in SOURCES:
        error = json.loads((run_dir / error_file).read_text(encoding="utf-8"))
        out["sources"][key] = _source_summary(error, audit, audit_key, truth_path)
    mission = run_dir / "mission.json"
    if mission.exists():
        record = json.loads(mission.read_text(encoding="utf-8"))
        out["mission"] = {
            "outcome": record["outcome"],
            "duration_s": round(float(record["duration_s"]), 1),
            "waypoints": int(record["waypoints"]),
            "recovery_counts": record.get("recovery_counts", {}),
        }
    return out


def build(openloop: Path, mission: Path, recorded: str, environment: str) -> dict:
    return {
        "simulated": True,
        "description": (
            "Simulated localisation accuracy on gz-sim, against the simulator's ground truth. "
            "One recorded run each; the README tables are rendered from this file."
        ),
        "environment": environment,
        "recorded": recorded,
        "runs": {
            "openloop": dict(command="make openloop", **summarise_run(openloop)),
            "mission": dict(command="make eval", **summarise_run(mission)),
        },
    }


def _fmt(value: Optional[float], unit: str = "", digits: int = 3, signed: bool = False) -> str:
    if value is None:
        return "n/a"
    text = ("%+." if signed else "%.") + str(digits) + "f"
    return (text % value) + unit


def render_table(run: dict) -> str:
    lines = [
        "| Source | ATE RMSE | ATE max | Yaw RMSE | Path vs truth | Mean NEES |",
        "|---|---|---|---|---|---|",
    ]
    for key, label, _error_file, _audit_key in SOURCES:
        s = run["sources"][key]
        lines.append("| %s | %s | %s | %s | %s | %s |" % (
            label,
            _fmt(s["ate_rmse_m"], " m"),
            _fmt(s["ate_max_m"], " m"),
            _fmt(s["yaw_rmse_deg"], " deg", 2),
            _fmt(s["path_vs_truth_pct"], "%", 1, signed=True),
            _fmt(s["nees_mean"], "", 2),
        ))
    return "\n".join(lines)


def render(reference: dict) -> Dict[str, str]:
    return {name: render_table(reference["runs"][name]) for name in RUNS}


def readme_blocks(text: str) -> Dict[str, str]:
    """The tables between <!-- gz-results:NAME --> and <!-- /gz-results:NAME -->."""
    out = {}
    for name in RUNS:
        match = re.search(r"<!-- gz-results:%s -->\n(.*?)\n<!-- /gz-results:%s -->" % (name, name),
                          text, re.S)
        if match:
            out[name] = match.group(1).strip()
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    b = sub.add_parser("build", help="Summarise two run directories into the reference file.")
    b.add_argument("--openloop", type=Path, required=True)
    b.add_argument("--mission", type=Path, required=True)
    b.add_argument("--recorded", required=True, help="Date of the runs, YYYY-MM-DD.")
    b.add_argument("--environment", required=True, help="Simulator and ROS versions the runs used.")
    b.add_argument("--output", type=Path, default=REFERENCE)
    r = sub.add_parser("render", help="Print the README tables for the reference file.")
    r.add_argument("--input", type=Path, default=REFERENCE)
    args = parser.parse_args()

    if args.command == "build":
        reference = build(args.openloop, args.mission, args.recorded, args.environment)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(reference, handle, indent=2)
            handle.write("\n")
        print("wrote %s" % args.output)
        return 0

    reference = json.loads(args.input.read_text(encoding="utf-8"))
    for name, table in render(reference).items():
        print("<!-- gz-results:%s -->\n%s\n<!-- /gz-results:%s -->\n" % (name, table, name))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
