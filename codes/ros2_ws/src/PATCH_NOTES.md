# Patch Notes - G1 A-to-B Navigation Benchmark

## Core architecture
- `g1_description_ros2/urdf/g1_nav.urdf` turns the full G1 into a **planar navigation platform**:
  - fixed humanoid joints for stable simulation
  - synthetic `base_link` collision footprint for Nav2 benchmarking
  - planar motion plugin for `/cmd_vel -> /odom`
  - simulated MID-360-style 3D lidar on `/livox/points`
  - simulated D435i-style depth camera on `/camera/depth/*`
- `g1_gazebo/launch/spawn_g1.launch.py` publishes robot state, spawns G1 into an existing Gazebo world, and converts `/livox/points` to `/scan`.
- `g1_cogar_nav_benchmark/launch/k3_nav_bringup.launch.py` is now the **main full-stack benchmark launch**:
  - scenario-driven world selection
  - G1 spawn at scenario start pose
  - perfect simulator baseline localization via static `map -> odom`
  - map server + Nav2 navigation servers
  - Braitenberg reflex + subsumption mux + benchmark logger
  - optional dynamic obstacle commander for the blockage scenario

## Built-in planners
- Fully wired and runnable: `dwb`, `mppi`
- Optional/external: `teb` (requires a Humble-compatible TEB Nav2 plugin already installed in the lab)

## Benchmark automation
- `scripts/run_one.sh` launches the full stack, waits for Nav2, runs the benchmark, and writes per-run metrics.
- `scripts/run_all_benchmarks.sh` runs all scenarios across all enabled planners and aggregates the results.

## Deliverable alignment
This patch is scoped to the assignment requirement of **standalone point-to-point indoor navigation benchmarking**. It intentionally treats G1 as a navigation platform in Gazebo rather than implementing full humanoid balance / locomotion control.

## Latest benchmark patch
- repeated-run execution now supports `RUNS`, `BASE_RESULTS_DIR`, `SCENARIOS`, and `PLANNERS`;
- raw runs are kept separate from aggregate statistics and plots;
- DWB and MPPI now use aligned robot footprint, frame, tolerance, costmap, and velocity limits for fairer comparison;
- Nav2 behavior-tree XML paths are resolved from the package at launch time instead of a user-specific home directory;
- summary CSV now includes mean/std-ready metrics for completion time, path length, path efficiency, near-collision events, recovery count, clearance, and multi-goal progress;
- plot generation is available through `tools/plot_results.py` and is called automatically by the batch script.
