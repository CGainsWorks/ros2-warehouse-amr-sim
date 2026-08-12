#!/usr/bin/env python3
"""Build the recovery behaviour matrix from an induced-failure run.

Takes the fault log written by ``fault_injector`` and the mission record written
by ``mission_runner``, correlates them in time, and reports which recovery the
behaviour tree actually selected for each induced failure and whether the robot
carried on afterwards.

    python analysis/recovery_matrix.py \
        --faults results/faults.json \
        --mission results/mission.json \
        --output results/recovery_matrix.json

CORRELATION IS BY TIME WINDOW, AND THAT IS A REAL LIMITATION

A fault is credited with the recoveries that fire within ``--window`` seconds
after it. Nothing in Nav2 tags a recovery with the reason it ran, so this is
attribution by adjacency rather than by causation. With faults spaced 20+
seconds apart, as ``fault_schedule.yaml`` sets them, ambiguity is rare — but a
recovery that fires 9 seconds after a fault might be responding to that fault or
to ordinary aisle traffic, and this script cannot tell the difference. Recoveries
falling in no window at all are reported separately as spontaneous, and a run
with many spontaneous recoveries is telling you the stack is unhappy for reasons
the fault schedule did not cause.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Optional, Sequence

# What each fault is supposed to provoke. Used only to flag surprises in the
# report; it never changes the recorded outcome.
EXPECTED = {
    "block_aisle": ["clear_local_costmap", "clear_local_costmap_context", "spin", "back_up", "wait"],
    "localisation_jump": ["clear_local_costmap", "clear_global_costmap", "spin"],
    "kidnap": ["clear_local_costmap", "clear_global_costmap", "spin", "back_up"],
    "relocalise": ["spin", "wait"],
    "sensor_dropout": ["wait", "clear_local_costmap"],
}


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def build_matrix(faults: Sequence[dict], events: Sequence[dict], window: float = 20.0) -> dict:
    """Attribute recovery events to the fault that most likely provoked them."""
    rows: List[dict] = []
    claimed = set()

    ordered = sorted(faults, key=lambda f: float(f["t"]))
    for index, fault in enumerate(ordered):
        start = float(fault["t"])
        # A window never extends past the next fault, so one fault cannot be
        # credited with the recoveries provoked by its successor.
        next_start = float(ordered[index + 1]["t"]) if index + 1 < len(ordered) else float("inf")
        end = min(start + window, next_start)

        matched = []
        for position, event in enumerate(events):
            t = float(event["t"])
            if start <= t < end:
                matched.append(event)
                claimed.add(position)

        kinds = [e["recovery"] for e in matched]
        expected = EXPECTED.get(fault["type"], [])
        rows.append(
            {
                "fault": fault["type"],
                "at_s": start,
                "detail": fault.get("detail", ""),
                "recoveries": kinds,
                "recovery_count": len(kinds),
                "first_recovery_latency_s": (
                    round(float(matched[0]["t"]) - start, 2) if matched else None
                ),
                "expected": expected,
                # A fault that provoked nothing at all is the interesting case:
                # either the stack absorbed it without needing recovery, which is
                # a good result, or the fault did not actually take effect, which
                # is a broken test. The two look identical here and have to be
                # separated by reading the run.
                "unexpected": sorted(set(kinds) - set(expected)) if expected else sorted(set(kinds)),
                "provoked_nothing": len(kinds) == 0,
            }
        )

    spontaneous = [e for position, e in enumerate(events) if position not in claimed]

    return {
        "window_s": window,
        "faults": rows,
        "spontaneous_recoveries": spontaneous,
        "totals": {
            "faults_injected": len(rows),
            "faults_provoking_recovery": sum(1 for r in rows if not r["provoked_nothing"]),
            "recoveries_attributed": sum(r["recovery_count"] for r in rows),
            "recoveries_spontaneous": len(spontaneous),
        },
    }


def _read_track(path: str) -> List[tuple]:
    import csv

    with open(path, newline="", encoding="utf-8") as handle:
        return [(float(r["t"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(handle)]


def localisation_response(
    faults: Sequence[dict],
    amcl_path: str,
    truth_path: str,
    settle: float = 0.25,
    horizon: float = 25.0,
) -> List[dict]:
    """Per fault: pose error just before, the peak after, and time to resettle.

    This is what separates the two readings of "provoked no recovery". A fault
    the stack absorbed shows a real error spike that decays; a fault that never
    took effect shows no spike at all. The recovery-behaviour columns alone
    cannot tell those apart, and the first version of this report said so in a
    footnote rather than measuring it.
    """
    import bisect
    import math

    amcl = _read_track(amcl_path)
    truth = _read_track(truth_path)
    if not amcl or not truth:
        return []

    truth_t = [row[0] for row in truth]

    def error_at(t: float, x: float, y: float) -> float:
        i = min(bisect.bisect_left(truth_t, t), len(truth) - 1)
        return math.hypot(x - truth[i][1], y - truth[i][2])

    out = []
    t0 = amcl[0][0]
    for fault in faults:
        at = float(fault["t"]) + t0
        before, peak, peak_t, resettled = None, None, None, None
        samples = 0

        for t, amcl_x, amcl_y in amcl:
            err = error_at(t, amcl_x, amcl_y)
            if t <= at:
                before = err
                continue
            if t > at + horizon:
                break
            samples += 1
            if peak is None or err > peak:
                peak, peak_t = err, t - at
            if peak_t is not None and resettled is None and err <= settle:
                resettled = t - at

        out.append({
            "fault": fault["type"],
            "at_s": float(fault["t"]),
            "error_before_m": before,
            "error_peak_m": peak,
            "samples_in_window": samples,
            "peak_at_s": peak_t,
            "resettled_after_s": resettled,
            "settle_threshold_m": settle,
        })
    return out


def format_localisation_response(rows: Sequence[dict]) -> str:
    if not rows:
        return ""
    lines = ["", "Localisation response (AMCL against Gazebo ground truth)", ""]
    header = "%-20s %8s %9s %9s %11s" % ("fault", "at (s)", "before", "peak", "resettled")
    lines += [header, "-" * len(header)]
    for row in rows:
        before = "-" if row["error_before_m"] is None else "%.3f m" % row["error_before_m"]
        if not row.get("samples_in_window"):
            lines.append("%-20s %8.1f %9s %9s %11s"
                         % (row["fault"], row["at_s"], before, "no data", "-"))
            continue
        resettled = ("never" if row["resettled_after_s"] is None
                     else "%.1f s" % row["resettled_after_s"])
        lines.append("%-20s %8.1f %9s %9s %11s"
                     % (row["fault"], row["at_s"], before, "%.3f m" % row["error_peak_m"], resettled))
    lines += [
        "",
        "  'resettled' is the first time the error returns under %.2f m within %.0f s."
        % (rows[0]["settle_threshold_m"], 25.0),
        "  'no data' means AMCL published nothing in the window, which is itself",
        "  a result: the filter had stopped updating.",
        "  A fault with no spike was absorbed upstream or never took effect;",
        "  a spike that never resettles is one the stack could not undo.",
    ]
    return "\n".join(lines)


def format_report(matrix: dict, mission: Optional[dict] = None) -> str:
    lines = [
        "Recovery behaviour matrix",
        "=========================",
    ]
    if mission:
        lines += [
            "mission outcome    %s in %.1f s over %d waypoints"
            % (mission.get("outcome", "?"), mission.get("duration_s", 0.0), mission.get("waypoints", 0)),
            "",
        ]

    header = "%-20s %8s %10s  %s" % ("fault", "at (s)", "latency", "recoveries invoked")
    lines += [header, "-" * len(header)]

    for row in matrix["faults"]:
        latency = "-" if row["first_recovery_latency_s"] is None else "%.1f s" % row["first_recovery_latency_s"]
        recoveries = ", ".join(row["recoveries"]) if row["recoveries"] else "(none)"
        lines.append("%-20s %8.1f %10s  %s" % (row["fault"], row["at_s"], latency, recoveries))
        if row["provoked_nothing"]:
            lines.append(
                "%-20s %8s %10s  ! provoked no recovery — absorbed, or the fault did not take effect"
                % ("", "", "")
            )
        elif row["unexpected"]:
            lines.append("%-20s %8s %10s  ! unexpected: %s" % ("", "", "", ", ".join(row["unexpected"])))

    totals = matrix["totals"]
    lines += [
        "",
        "%d of %d faults provoked a recovery; %d recoveries attributed, %d spontaneous"
        % (
            totals["faults_provoking_recovery"],
            totals["faults_injected"],
            totals["recoveries_attributed"],
            totals["recoveries_spontaneous"],
        ),
    ]
    if totals["recoveries_spontaneous"]:
        lines.append(
            "  spontaneous recoveries fired outside any fault window; the stack was "
            "struggling for reasons the schedule did not cause"
        )
    return "\n".join(lines)


def build_matrix_from_runs(root: Path, window: float = 20.0) -> dict:
    """Aggregate one-fault-per-run directories into a single matrix.

    Each subdirectory is a complete mission that met exactly one induced
    failure, so its recovery events and its localisation trace belong to that
    fault and to nothing else.
    """
    runs = []
    for run_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        faults_file, mission_file = run_dir / "faults.json", run_dir / "mission.json"
        if not faults_file.exists() or not mission_file.exists():
            runs.append({"fault": run_dir.name, "status": "no result — the run did not reach the fault"})
            continue

        faults = load_json(str(faults_file)).get("faults", [])
        mission = load_json(str(mission_file))
        if not faults:
            runs.append({"fault": run_dir.name, "status": "the fault never fired"})
            continue

        inner = build_matrix(faults, mission.get("recovery_events", []), window=window)
        row = dict(inner["faults"][0])
        row["status"] = "measured"
        row["outcome"] = mission.get("outcome", "?")
        row["duration_s"] = mission.get("duration_s")
        row["detail"] = faults[0].get("detail", "")

        amcl, truth = run_dir / "amcl_pose.csv", run_dir / "ground_truth.csv"
        if amcl.exists() and truth.exists():
            try:
                row["localisation"] = localisation_response(faults, str(amcl), str(truth))[0]
            except (OSError, KeyError, ValueError, IndexError):
                pass
        runs.append(row)

    measured = [r for r in runs if r.get("status") == "measured"]
    return {
        "window_s": window,
        "isolated": True,
        "runs": runs,
        "totals": {
            "faults_run": len(runs),
            "faults_measured": len(measured),
            "faults_provoking_recovery": sum(1 for r in measured if not r.get("provoked_nothing")),
            "missions_completed": sum(1 for r in measured if r.get("outcome") == "succeeded"),
        },
    }


def format_isolated_report(matrix: dict) -> str:
    lines = [
        "Recovery behaviour matrix",
        "=========================",
        "one induced failure per run, so no fault is measured on a robot",
        "another fault has already broken",
        "",
    ]
    header = "%-20s %-10s %8s  %-26s %s" % ("fault", "mission", "latency", "recoveries invoked", "AMCL peak / resettled")
    lines += [header, "-" * len(header)]

    for row in matrix["runs"]:
        if row.get("status") != "measured":
            lines.append("%-20s %s" % (row["fault"], row.get("status", "?")))
            continue

        latency = ("-" if row.get("first_recovery_latency_s") is None
                   else "%.1f s" % row["first_recovery_latency_s"])
        recoveries = ", ".join(row["recoveries"]) if row["recoveries"] else "(none)"

        response = row.get("localisation") or {}
        if not response.get("samples_in_window"):
            localisation = "no AMCL data"
        else:
            resettled = ("never" if response.get("resettled_after_s") is None
                         else "%.1f s" % response["resettled_after_s"])
            localisation = "%.2f m / %s" % (response.get("error_peak_m") or 0.0, resettled)

        lines.append("%-20s %-10s %8s  %-26s %s"
                     % (row["fault"], row.get("outcome", "?"), latency, recoveries, localisation))

    totals = matrix["totals"]
    lines += [
        "",
        "%d of %d faults measured; %d provoked a recovery; %d missions still completed"
        % (totals["faults_measured"], totals["faults_run"],
           totals["faults_provoking_recovery"], totals["missions_completed"]),
        "",
        "  'AMCL peak' is the worst pose error against Gazebo ground truth in the",
        "  25 s after the fault; 'resettled' is when it first returns under 0.25 m.",
        "  A fault with no recovery and no error peak was absorbed upstream.",
    ]
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", help=(
        "Directory of per-fault run directories, one subdirectory per fault, as "
        "produced by scripts/run_faults.sh. Preferred: each fault is measured on "
        "a robot the other faults have not already broken."))
    parser.add_argument("--faults", help="results/faults.json from fault_injector.")
    parser.add_argument("--mission", help="results/mission.json from mission_runner.")
    parser.add_argument("--window", type=float, default=20.0, help="Attribution window, seconds.")
    parser.add_argument("--output", help="Write the matrix as JSON here.")
    parser.add_argument("--amcl", help="results/amcl_pose.csv, for the localisation response.")
    parser.add_argument("--truth", help="results/ground_truth.csv, for the localisation response.")
    args = parser.parse_args(argv)

    if not args.runs and not (args.faults and args.mission):
        parser.error("either --runs, or both --faults and --mission, is required")

    if args.runs:
        matrix = build_matrix_from_runs(Path(args.runs), window=args.window)
        print(format_isolated_report(matrix))
    else:
        faults = load_json(args.faults).get("faults", [])
        mission = load_json(args.mission)
        events = mission.get("recovery_events", [])

        matrix = build_matrix(faults, events, window=args.window)
        matrix["mission"] = {k: v for k, v in mission.items() if k != "recovery_events"}

        if args.amcl and args.truth:
            try:
                matrix["localisation_response"] = localisation_response(faults, args.amcl, args.truth)
            except (OSError, KeyError, ValueError) as exc:
                print("note: localisation response unavailable (%s)" % exc)

        print(format_report(matrix, mission))
        if matrix.get("localisation_response"):
            print(format_localisation_response(matrix["localisation_response"]))

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(matrix, indent=2), encoding="utf-8")
        print("\nwrote %s" % out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
