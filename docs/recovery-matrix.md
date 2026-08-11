# Recovery behaviour matrix

## Status

**The faults, the recovery ladder and the analysis are implemented; the run that
populates the results table has not been performed.** The table below is empty
for that reason rather than filled with plausible values. Generate it with:

```bash
docker compose -f docker/docker-compose.yml run --rm faults
```

or `make faults` in a sourced workspace. That writes
`results/recovery_matrix.json` and prints the table.

## The recovery ladder

From `amr_navigation/behavior_trees/navigate_w_recovery.xml`. Actions are in a
`RoundRobin`, not a `Sequence`: each failure runs the *next* action rather than
replaying the list from the start, so six retries walk through all five actions
once rather than clearing the local costmap six times.

The order is the design — cheapest and least disruptive first, and each step
addresses a different cause of failure.

| # | Action | Addresses | Why here in the order |
|---|---|---|---|
| 1 | `ClearEntireCostmap` (local) | A stale obstacle left by someone who has walked away; a smear from a momentary localisation jump between bays. | Cheapest possible. Costs nothing and does not move the robot. By far the most common cause. |
| 2 | `ClearEntireCostmap` (global) | The global planner has convinced itself the aisle is blocked. | Separated from step 1 rather than combined, because clearing the global costmap discards genuine knowledge about obstacles outside sensor range. Worth trying the local clear alone first. |
| 3 | `Spin` (90°) | Costmap and localisation both stale from one viewpoint. | First action that moves. Sweeps the 2D LiDAR across ignored space and — because the bays alias — gives AMCL a genuinely new measurement. Not a full 360°: in a 2.8 m aisle that costs several seconds and the second half observes nothing the first half did not. |
| 4 | `BackUp` (0.30 m at 0.05 m/s) | The robot is inside the inflation band, or nose-first into a pallet, so no forward trajectory scores. | Deliberately after the spin, because reversing is blind: there is no rear-facing sensor coverage below the 2D LiDAR plane. |
| 5 | `Wait` (5 s) | The aisle is blocked by a person or a forklift. | Last resort and the right answer more often than it looks. Every other recovery is noise against a moving blockage, and waiting is the only recovery that cannot make things worse. |

The whole `RoundRobin` sits inside a `ReactiveFallback` guarded by `GoalUpdated`,
so a robot mid-spin abandons the recovery immediately if new work arrives rather
than finishing the spin first.

`tests/test_config_consistency.py` asserts this ordering, and that the node names
match the ones `mission_runner` watches for — renaming a node in the XML without
updating the runner would silently produce an empty matrix.

## The induced faults

From `amr_bringup/config/fault_schedule.yaml`. Scheduled by **time, not robot
position**, so a run stays reproducible even when the robot's own behaviour
changes in response to earlier faults. If faults fired on position, a robot that
recovered differently would meet the next fault in a different state and
comparison between runs would be meaningless.

| t (s) | Fault | Mechanism | Meant to provoke |
|---|---|---|---|
| 12 | `block_aisle` | Teleport `aisle_pallet_0` into the robot's path via `/gazebo/set_entity_state`. | Local costmap invalidates the plan. Expect replan, escalating to a local clear. |
| 35 | `localisation_jump` | Publish a confident `/initialpose` exactly one racking bay (2.7 m) from truth. | The aliasing failure the environment is built around. The scan matches the wrong bay perfectly; only AMCL's global map can pull it back. |
| 70 | `kidnap` | Teleport the *robot* 5.4 m down the aisle without telling anything. | Harder than the pose jump: the estimate is right and the world is wrong. |
| 95 | `relocalise` | Call `/reinitialize_global_localization`. | Measures reconvergence time where every bay looks alike. |
| 120 | `sensor_dropout` | `SIGSTOP` gzserver for 4 s (`scripts/run_faults.sh`). | Costmap `expected_update_rate` watchdog and the scan matcher's rejection path. Expect the robot to slow or stop rather than continue on stale data. |

The dropout is done from the shell rather than from `fault_injector`, because
making a Gazebo sensor stop publishing means unloading its plugin, and that is
not recoverable within a run.

## Results

Populate from `results/recovery_matrix.json` after running `make faults`.

| Fault | Injected at (s) | First recovery latency (s) | Recoveries invoked | Mission continued |
|---|---|---|---|---|
| `block_aisle` | 12 | — | — | — |
| `localisation_jump` | 35 | — | — | — |
| `kidnap` | 70 | — | — | — |
| `relocalise` | 95 | — | — | — |
| `sensor_dropout` | 120 | — | — | — |

## How attribution works, and its limitation

Recovery events come from `/behavior_tree_log`, on the `IDLE → RUNNING`
transition — the moment a node is actually invoked. They are read from the BT
log rather than inferred from robot motion, because inference cannot distinguish
a `Spin` recovery from a legitimate in-place turn onto a new heading, and which
recovery fired is the entire content of the matrix.

A fault is credited with recoveries firing within a 20 s window after it, capped
at the next fault's timestamp so one fault cannot be credited with its
successor's consequences.

**This is attribution by adjacency, not by causation.** Nothing in Nav2 tags a
recovery with the reason it ran. With faults 20+ seconds apart ambiguity is rare,
but a recovery firing 9 s after a fault might be responding to that fault or to
ordinary aisle traffic, and the script cannot tell. Recoveries falling in no
window are reported separately as *spontaneous*, and a run with many of them is
telling you the stack is unhappy for reasons the schedule did not cause.

One more case worth reading carefully: a fault that provokes **no** recovery is
reported explicitly. That is either the stack absorbing it cleanly, which is a
good result, or the fault failing to take effect, which is a broken test. The two
look identical in the output and have to be separated by reading the run log.
