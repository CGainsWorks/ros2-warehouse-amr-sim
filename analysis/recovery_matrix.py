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


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--faults", required=True, help="results/faults.json from fault_injector.")
    parser.add_argument("--mission", required=True, help="results/mission.json from mission_runner.")
    parser.add_argument("--window", type=float, default=20.0, help="Attribution window, seconds.")
    parser.add_argument("--output", help="Write the matrix as JSON here.")
    args = parser.parse_args(argv)

    faults = load_json(args.faults).get("faults", [])
    mission = load_json(args.mission)
    events = mission.get("recovery_events", [])

    matrix = build_matrix(faults, events, window=args.window)
    matrix["mission"] = {k: v for k, v in mission.items() if k != "recovery_events"}

    print(format_report(matrix, mission))

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(matrix, indent=2), encoding="utf-8")
        print("\nwrote %s" % out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
