# Recovery behaviour matrix

## How this is run, and why it is five runs

**One induced failure per run.** `scripts/run_faults.sh` starts the simulation
once per fault, injects exactly that fault at the same mission time in each, and
aggregates the five runs into one matrix:

```bash
docker compose -f docker/docker-compose.yml run --rm faults
```

or `make faults` in a sourced workspace. That writes
`results/recovery_matrix.json`, a `results/faults/<fault>/` directory per run,
and prints the table below.

This started as one mission carrying all five faults, which is cheaper and does
not work. Two things went wrong, and both are worth keeping in mind for any
fault-injection harness:

**A fault that ends the run hides every fault after it.** The sensor dropout was
scheduled at 120 s, behind a kidnap the stack is not designed to survive. The
mission aborted at 100 s. The dropout had never executed once, and nothing in
the output said so — the matrix simply listed four faults instead of five.

**A fault lands on a robot the previous fault already broke.** Measured in one
combined run, the dropout at 55 s was applied to a filter that the pose jump at
35 s had left 8 m out. Its "recovery latency" was mostly the previous fault
still unwinding.

This is the same argument as the open-loop localisation harness: to measure a
thing, stop the other things feeding into it. Five short runs cost more
wall-clock than one long one and are the only version that answers the question.

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
position**, so a run stays reproducible. If faults fired on position, a robot
that recovered differently would meet the fault in a different state and the
comparison between runs would be meaningless.

Because each fault gets its own run, the schedule's `at:` times are overridden
to a single common time (25 s) so every fault meets a robot in the same state:
mid-leg on the first traverse, with room ahead to recover. The order in the file
still matters for reading it, and runs least to most severe.

| Fault | Mechanism | Meant to provoke |
|---|---|---|
| `block_aisle` | Teleport `aisle_pallet_0` into the robot's path via `/gazebo/set_entity_state`. | Local costmap invalidates the plan. Expect replan, escalating to a local clear. |
| `localisation_jump` | Publish a confident `/initialpose` exactly one racking bay (2.7 m) from truth. | The aliasing failure the environment is built around. The scan matches the wrong bay perfectly; only AMCL's global map can pull it back. |
| `sensor_dropout` | Tell `scan_relay` to stop forwarding for 4 s. | Costmap `expected_update_rate` watchdog and AMCL coasting on odometry. Expect the robot to slow or stop rather than continue on stale data. |
| `kidnap` | Teleport the *robot* 5.4 m down the aisle without telling anything. | Harder than the pose jump: the estimate is right and the world is wrong. **Not expected to be survivable** — see below. |
| `relocalise` | Call `/reinitialize_global_localization`. | Measures reconvergence time where every bay looks alike. This is the manual substitute for the automatic particle injection that is disabled on measured evidence. |

### The kidnap is expected to fail, and that is a deliberate trade

`amcl.yaml` sets `recovery_alpha_slow: 0.0` and `recovery_alpha_fast: 0.0`,
which disables random particle injection. Without injection AMCL has no
mechanism for recovering from a true kidnapping: every particle is in the wrong
place and nothing introduces new ones.

That was not an oversight. Injection was enabled and **measured**, and it made
localisation roughly six times worse — 1.32 m to 12.47 m ATE — because in a map
that repeats every 2.7 m the injected particles land on plausible wrong bays and
get reinforced. The trade is: give up automatic kidnap recovery, which a
warehouse AMR on a known floor almost never needs, to keep the localisation that
it needs every second. `relocalise` exists as the manual trigger for the rare
case, and this matrix measures how long that takes.

### The dropout needs a relay, and used to be faked

A Gazebo sensor cannot be stopped and restarted inside a run: making it stop
publishing means unloading its plugin, which is not recoverable. So the fault
run — and only the fault run — sets `use_scan_relay:=true`, which moves the
LiDAR onto `scan_raw` and interposes `amr_bringup`'s `scan_relay`. The injector
tells the relay to stop forwarding and restores it on a timer.

The previous implementation was `pkill -STOP -f gzserver`, suspending the whole
simulator. Under `use_sim_time` that stops the clock, so every node in the stack
stops with it and nothing observes a dropout at all. It tested nothing, and it
had never run anyway because it sat behind the kidnap.

## Results

One run per fault, each injected at t = 25 s. `results/recovery_matrix.json`.

| Fault | Recovery latency | Recoveries invoked | AMCL peak error | Resettled | Mission\* |
|---|---|---|---|---|---|
| `block_aisle` | — | (none) | 7.30 m | never | timeout |
| `localisation_jump` | — | (none) | 10.27 m | never | aborted |
| `sensor_dropout` | — | (none) | **0.07 m** | **0.3 s** | timeout |
| `kidnap` | — | (none) | 13.24 m | never | aborted |
| `relocalise` | 5.7 s | spin, back_up, wait | no AMCL data | — | aborted |

**AMCL peak** is the worst pose error against Gazebo ground truth in the 25 s
after the fault; **resettled** is when it first comes back under 0.25 m.

### \* The mission column is not attributable to the fault

Fault runs put `scan_relay` in the LiDAR path so the dropout can be induced at
all. A control run — **relay in the path, fault scheduled so far out that it
never fires** — aborted at 222 s, where the same mission without the relay
succeeds in 156 s.

So the relay on its own is enough to lose the mission. Every mission outcome in
the table above is confounded by it, and none of them can be read as "the fault
did this". The recovery and AMCL columns are still attributable, because those
are measured against the fault's own timestamp within a run.

This is an unsatisfying place to leave it and it is where it is left. The
suspected cause is that a Python node in a 10 Hz path on a machine already
running Gazebo, Nav2 and the perception stack adds enough jitter to matter — the
same starvation that made Nav2's bringup fail earlier in this work until
perception was deferred. The fix is a C++ relay, or an induced dropout that does
not need one. `scan_relay` now logs forwarded and dropped counts every 10 s so
the next person has the throughput data; the first attempt logged only at
shutdown, and the process is killed rather than asked to exit, so that number
was never once printed.

What the control does establish is that **the relay is not in the path for any
other run in this repository**, which is why the localisation figures elsewhere
are unaffected by it. That was a deliberate design choice and this result is the
argument for it.

### Reading the rest

**`sensor_dropout` is the clean result.** Blinding the LiDAR for 4 s moved AMCL
by 0.07 m and it was back under 0.25 m within 0.3 s of the scans returning. No
recovery behaviour was needed. The EKF coasting on wheel odometry and IMU across
a four-second gap is exactly what the fusion is there for, and it did it without
the navigation stack noticing.

**`kidnap` is a failure by design, and the trade is documented above.** A 13.24 m
error that never resettles is what disabling random particle injection buys you.
The alternative was measured: injection enabled made localisation roughly six
times worse in normal driving.

**`localisation_jump` should be read next to `kidnap`.** A 2.7 m pose error —
exactly one racking bay — produced a 10.27 m peak that never resettled. The
aliasing hazard this environment is built around is real: once the filter is
convinced it is in the wrong bay, the scan agrees with it, and the run does not
come back. `relocalise` is the only route out, which is what makes it the fifth
fault rather than a curiosity.

**`relocalise` is the one fault that provoked the recovery ladder**, at 5.7 s,
and walked spin → back_up → wait in order. AMCL stopped publishing during it,
which is why there is no error figure: the filter had scattered its particles
and had not yet reconverged, and the robot stopped moving, so there were no
updates to publish.

**`block_aisle` needed a fix before it measured anything.** It teleports a
0.8 m pallet into the aisle, and it did so at y = 0.0, dead centre — which
leaves a zero-cost lane 0.06 m wide and is not a replanning test but an
impassable-aisle test. It is now offset to y = 0.45, leaving 0.51 m. The 7.30 m
AMCL peak that remains is not explained and is the first thing to look at if
this work continues.

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
