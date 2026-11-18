# Fixed-map localisation plan — `feature/localisation`

## Objective

Estimate the robot's global planar pose `(x, y, yaw)` in the known 2.500 m × 2.500 m Unibots arena from `/scan`, without using simulated ground truth or wheel odometry as an estimator input. Start with stationary initialisation; then maintain reliable estimates while moving in the presence of three other robots that occlude the arena walls. Keep the design open to future odometry/IMU prediction without implementing fusion now.

## Current conventions and status

- The map origin is the arena centre. `arena.yaml` sets `forward_axis: y` and `forward_sign: +1`; the home wall is at `y = -1.25 m`, with forward into the arena along `+y`.
- `sensors.yaml` sets our footprint to `0.200 × 0.200 m`. Its LiDAR mounting relative to `base_footprint` is `(0.020, 0, 0.144) m` and yaw `0°`, following upstream `main`'s forward-facing mount (provisional position; yaw still to be confirmed on hardware with `lidar_viewer`). The simulator, `lidar_tf.py`, both launch files and localisation preprocessing all read this one file.
- Our current nominal start is `(0, -1.15, +90°)` in map coordinates: centred at the home wall, rear edge touching. `match.yaml` holds separate `start` (estimator prior) and `end` (return-home target) offsets relative to this nominal pose; `frames.match_pose()` resolves both, and both are currently `(0, -1.15, +90°)`. The node exposes the target as `home_target`; nothing navigates to it yet.
- `frames.py` supplies the fixed walls. `scan_matcher.py` performs robust point-to-wall optimisation; `localisation_node.py` subscribes to `/scan` and publishes `/localisation/pose` and `/localisation/valid`. It does **not** subscribe to `/odom`, fuse an IMU, or publish `map -> odom`.
- Square-footprint collision checking and a LiDAR raycaster with the shared mounting are implemented in `robot_sim`.
- Upstream `main` (own strafing firmware, `lidar_viewer`) is merged.
- Milestone A is complete: six ideal-simulation start-pose benchmarks (nominal, ±0.20 m, +0.28 m, two heading/inward offsets) recovered truth to below 1e-6 m and 1e-5° from the unchanged nominal prior. Rerun at 0° LiDAR yaw for nominal, +0.20 m and the combined offset with the same result. These scans are noise-free; noise, dropouts, occlusions and moving opponents remain untested.
- Earlier checks: 40 passing localisation tests; a live stationary 20 Hz run returned 160/160 valid estimates and zero error against `/odom` at the nominal start. This verifies wiring and geometry, **not** convergence from an unknown pose. `/odom` currently equals simulator truth.

## Key design choices

1. **Known map, not SLAM.** Arena wall positions remain fixed; only the robot pose is estimated. Do not give the optimiser dynamic robot locations or simulator truth.
2. **Prior versus truth.** Initialise the estimator from the configured home-wall assumption, not the simulator's actual overridden starting pose. Later connect `match.yaml` to the prior while keeping test truth independently configurable.
3. **Walls provide geometric evidence.** For candidate pose `q`, a beam has predicted wall range `r_wall(q, beam)`. An observed range much shorter than this is *potentially occluded*; it should not be forced onto a wall. A return significantly longer than the predicted wall range is inconsistent with the candidate map pose, except for measurement error, missed returns or wall-model errors. Use tolerances and distinguish missing returns from measured ones.
4. **Never filter by a universal near-range cutoff.** True walls can be close when the robot drives near them; classify relative to predicted wall distance. Require enough geometrically independent wall support to avoid an 'everything is occluded' solution.
5. **Pure geometry separated from ROS.** Preserve the standalone matcher and LiDAR/raycast functions. ROS nodes orchestrate messages; benchmarking reads ground truth only on a separate evaluation path.
6. **Frequency is measured.** The current simulated `/scan` rate is 20 Hz. Match each new scan at that rate if processing can keep up, and measure calculation latency, scan age and dropped/stale messages under motion. Avoid accumulating a backlog of outdated scans.
7. **No IMU fusion in this phase.** Keep motion prediction logically separate from scan measurement updates, so a future odometry/IMU predictor can be inserted without rewriting the matcher. Do not introduce unused prediction-framework scaffolding.

## Milestone A — Reproducible unknown-pose evaluation (done)

Make **four immediate code changes**:

1. `ws/src/robot_sim/robot_sim/sim_node.py`: support an optional, explicit simulator-only starting-pose override (map `x`, `y`, yaw in degrees). If omitted, retain `world['start']` from the current config-derived nominal pose. Validate finite values and reject a starting footprint penetrating walls; ensure no truth is passed to the estimator.
2. `ws/src/robot_bringup/launch/sim.launch.py`: expose/forward the override to the simulator without breaking other worlds, drive options, TF publishers or the normal launch path. Use unambiguous units and an omitted/default setting that truly means 'use world start'.
3. `ws/src/robot_sim/test/test_sim_motion.py`: test the default, lateral overrides, valid offset/heading cases away from walls, invalid configurations and collisions at the start. Test whichever start-pose parsing helper is added, independently of ROS where feasible.
4. `scripts/benchmark_localisation.py` (new): a standalone, bounded ROS 2 evaluation tool. Subscribe to `/localisation/pose`, `/localisation/valid` and simulator `/odom`, associate samples by timestamp or a documented tolerance, compute position and wrapped-yaw errors, and output a clear per-run summary plus optional CSV. **Never** publish odometry to, or otherwise feed ground truth into, the localisation node. It should work against separately started simulator and localisation processes; no automatic spawning of conflicting simulator instances in this first version.

Acceptance: leave the localisation prior unchanged when overriding simulator truth; run several valid starts within the ±0.28 m lateral range and some inward/heading offsets; measure initial convergence and all-scan errors. Include a control trial where the initial guess differs from actual truth. Check the simulator remains usable on port 8080 and unrelated worlds still work.

## Milestone B — Model-aware occlusion handling

- Establish a benchmark baseline with the existing point-to-wall solver first.
- Compare it with a wall-raycast observation model using `r_wall(q, alpha)` for each beam. For a return close to predicted wall range, use robust wall agreement; for a shorter return, treat it as potential occlusion and downweight/reject; for a longer return, diagnose inconsistency rather than treating it as a valid wall hit.
- Do **not** minimise a one-sided cost that is zero whenever all beams are classified as occluded: require minimum inlier counts, nonparallel wall support, informative angular coverage and a reasonable prior/jump gate. Return `valid=false` when the pose is underconstrained.
- Consider a hybrid method: use existing point-to-wall matching to generate candidates, then score/verify them with the predicted-range model. Benchmark computational cost before replacing the existing optimiser.
- Add tests for robots obscuring the opposite wall, two adjacent walls, near-wall operation, missing returns, spurious long ranges, noise and erroneous candidate poses.

Acceptance: no false-confidence estimate when too few walls remain observable; maintain good pose accuracy where independent walls are visible; measure the speed/robustness trade-off at a 20 Hz input rate.

## Milestone C — Three moving robot occluders in simulation

- Model exactly three additional bodies, each a rotating 0.200 × 0.200 m square. They are **dynamic simulator obstacles**, not extra localisation landmarks and not independent ROS robots unless a later test requires that.
- Use deterministic random seeds, bounded speeds and turn rates, and collision-checked motion against arena walls, our robot and each other. On blocked moves, turn/choose a new velocity instead of tunnelling through geometry. Maintain reproducible initial placements with no overlaps.
- At every scan, raycast against the fixed walls **and** the current other-robot outline segments. Keep the known map in the estimator wall-only; other robots are unmodelled occluders from its point of view.
- Show all squares in the existing simulator browser, visibly distinguished from our robot. Report their positions in browser/debug ground truth only, not as localisation inputs.
- Make the robot count and seed configurable, with `count=0` preserving the current baseline. Tests cover motion bounds, collision avoidance, ray occlusion and reproducibility.

Likely files in this later milestone: `robot_sim/robot_sim/sim_node.py`, `robot_sim/robot_sim/lidar.py` and/or a small plain-Python dynamics helper, `robot_sim/robot_sim/web/index.html`, with associated simulator tests. Implement only after the unknown-pose benchmark exists, so failures are attributable.

## Milestone D — Fast, continuous LiDAR-only tracking

- Start from the last accepted LiDAR pose and use scan timestamps (`dt`). Measure how much the robot moves between scans at varied commanded speeds and angular velocities.
- Investigate a simple constant-velocity pose prediction derived only from the last two valid LiDAR estimates; use a speed- and `dt`-aware innovation gate. Compare against the existing previous-pose-only baseline.
- On poor wall visibility, mark estimates invalid and retain the last accepted pose with an explicit age; do not silently publish it as current ground truth. On recovery, broaden candidate initialisation without jumping to a symmetric square-arena solution.
- Separate prediction from LiDAR measurement matching in interfaces, but do not implement IMU or wheel-odometry fusion now. A future predictor can accept other sensor inputs without changing `scan_matcher.py`.
- Calibrate or clearly label published covariance; current `initial_pose_sigma` is a fixed prior, not an observed posterior variance.

Acceptance: measure validity rate, localisation error, latency and relocalisation after temporary occlusion at low and high speeds.

## Milestone E — Real robot validation

- Re-measure the 20 mm LiDAR forward offset and verify yaw against the actual installed sensor and TF.
- Compare physical wall ranges to the geometry, including ranges near walls, chassis occlusion and other robots. Keep physical motor commands separate from localisation testing until stationary checks pass.
- Confirm no conflicting `map -> odom` TF publisher runs alongside Jack's SLAM. Integrate with his navigation only after the independent estimator has measurable reliability.

## Evaluation protocol and provisional targets

- **Scenarios:** nominal start, legal lateral extremes `±0.28 m`, intermediate offsets, inward-translated starts, small heading errors with physically collision-free positions, moving paths, increasing obstacle occlusion.
- **Metrics:** valid-initialisation fraction, time to first valid estimate, positional error/RMSE and p95, wrapped heading error/RMSE and p95, largest pose jump, wall inliers and wall diversity, scan rate, solver p50/p95 latency and stale-scan fraction.
- **Provisional clean-simulation targets:** position error < 0.03 m, yaw error < 3°, ≥95% successful initialisations, first reliable estimate within ~1 s, and processing fast enough to keep up with the 20 Hz scan stream. Treat these as acceptance criteria to measure, not established results.
- **Reproducibility:** record arena/sensor config, simulator world, start pose, random seed, noise/occlusion setting, test duration and code revision for each run. Report failure cases instead of weakening tests.

## Scope boundaries

Do not build SLAM, learn a new map, change Jack's navigation code, add IMU fusion, or use `/odom` as an estimator input. Do not push or merge until changes are reviewed and tested. Preserve existing other-world simulation behaviour and the current stable nominal-start benchmark.
