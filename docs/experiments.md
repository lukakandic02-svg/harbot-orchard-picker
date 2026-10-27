# Experiments

All experiments run in Gazebo through one script,
`experiments/run_experiment.sh <profile>`. For each mission it stops the
previous world, starts a new one, waits until Gazebo, MoveIt and all
controllers are up, starts the robot node(s) with the profile's parameters,
records the run, computes the metrics and leaves a fresh world running for
the next profile. All times are simulated time.

With `REPORT_SESSION_DIR=<dir>` a run also records the trajectories of both
robots (20 Hz, in simulator XY and in map terms), a rosbag, a screen video,
the exact command line and all ROS parameters.

## 1. Single-robot study (`sr_*` profiles)

40 missions in 18 profiles, 12 targets on the ladder graph, varying the
routing strategy, basket capacity, number of targets, speed, target layout
and continuous motion. All 40 missions picked every target without a safety
stop (max tilt 0.66 deg); 34 of 418 grasp attempts failed and were retried
successfully within the limit of 3 attempts. Two results were used for the
two-robot experiments:

* distance driven is a much more stable metric than duration (0.33 % vs
  2.45 % spread within a profile);
* about 750 s per mission the base is standing still (arm, planner,
  alignment), independent of driving speed, so speeds above 0.44 m/s bring
  little.

## 2. Work split between two robots (`dual_equal` ... `dual_speed_zone80`)

Separate zones, 10 targets, robot B at full or half speed, and the faster
robot given 50 %, 60 %, 70 % or 80 % of the targets. The hypothesis was that
the faster robot should get a bigger zone. The results showed the opposite:
as its share went from 50 % to 80 %, the difference between the finish times
of the two robots grew from 63 s to 883 s, and productivity fell from 47.3 to
29.0 targets/h. Each target takes 100-200 s of approach and manipulation that
does not depend on driving speed, which is more than the driving time saved.
With shortcuts the distance was 14 % shorter on average, but the slower
robot was still the bottleneck.

## 3. Topology matrix (`dual_topology_<topology>_seed<seed>`)

The main experiment is a full factorial matrix, run unattended on frozen code
with `experiments/run_topology_matrix.sh`.

| Factor | Levels |
|---|---|
| Workspace topology | line, M1, M2, M3, full (see [topologies.md](topologies.md)) |
| Target split (seed) | 37, 41, 45, 54, 162 |
| Repetitions | 3 |
| Missions | 75 |
| Targets per robot | 6 of 12 |
| Basket capacity | 6 fruits |
| Speed (both robots) | 0.44 m/s |
| Target selection | PROST |
| Time limit per mission | 2500 s |

The five seeds were chosen from about 300 candidates to maximise the minimum
distance between targets of the same robot (5.40 m for all five). The runner
saves the md5 of every file that affects behaviour and does not start a new
pass if any of them changed, so all 75 missions use the same code.

`results/topology_matrix.csv` has one row per mission. To regenerate the
summary and the figure:

```bash
tools/analysis/summarize_matrix.py --summary <pass dirs>/overnight_summary.txt \
    --out results/topology_matrix.csv
tools/analysis/plot_results.py results/topology_matrix.csv docs/media/results.png
```

### Results

![Topology matrix results](media/results.png)

| Topology | Success | Distance, both robots [m]* | Makespan median [s]* | Deadlock events |
|---|---|---|---|---|
| line | 2/15 (13 %) | 478 | 1494 | 437 |
| M1 | 11/15 (73 %) | 452 | 1526 | 170 |
| M2 | 13/15 (87 %) | 382 | 1487 | 190 |
| M3 | 10/15 (67 %) | 319 | 1465 | 168 |
| full | 14/15 (93 %) | 297 | 1279 | 21 |

\* successful missions only (mean distance, median makespan).

* Success goes from 13 % (line) to 93 % (full). Line vs. full is highly
  significant (Fisher exact p = 2.2e-5), and so is the first step, line vs.
  M1 (p = 0.003).
* Distance decreases monotonically with connectivity (Spearman rho = -0.95
  over the 50 successful missions), 38 % from line to full.
* Makespan drops clearly only for `full`. Larger graphs make each PROST
  cycle slower (20 s on line vs. 46 s on M3), which cancels much of the
  saved driving time. In missions without deadlocks the makespan does
  decrease monotonically.
* 23 of the 25 failures were bottleneck deadlocks that could not be
  resolved, all in 6 seed/topology cells (0/3 in every repetition). None of
  the failures was a safety failure: no tilt aborts in 75 missions, minimum
  distance between the robots 0.647 m (median 0.745 m, close to the 0.75 m
  threshold), and the grasp failure rate (8.2 % of attempts, all retried
  successfully) did not depend on the topology.

The full analysis with statistics is in chapter 5 of the thesis.
