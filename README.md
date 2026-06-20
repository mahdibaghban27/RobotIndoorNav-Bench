# Unitree G1 A-to-B Navigation Benchmark

A standalone indoor A-to-B navigation benchmark for a simulated **Unitree G1 EDU** robot using **ROS 2 Humble**, **Gazebo Classic**, **Nav2**, and **RViz2**.

The project compares three Nav2 local planner/controller profiles - **DWB**, **MPPI**, and **Regulated Pure Pursuit (RPP)** - under identical indoor benchmark conditions. A common **ThetaStar** global planner is used in all runs. The system also includes execution monitoring, Nav2 recovery, CSV telemetry, automated result aggregation, and a hard-wired **Braitenberg-inspired safety reflex** with subsumption-based command arbitration.

> Full technical report: [COGAR G1 Navigation Project Report](docs/COGAR_G1_Navigation_Project_Report.pdf)

---

## Highlights

- Unitree G1 EDU simulation with a simplified 29-DOF humanoid motion model
- Livox MID-360-style LiDAR and Intel RealSense D435i-style depth sensing
- Fixed known maps and simulator-provided baseline localization
- ThetaStar global planning
- DWB, MPPI, and RPP local planner/controller profiles
- Five evaluated scenarios and 45 benchmark runs
- Static and dynamic obstacle avoidance
- Nav2 progress monitoring and recovery behaviors
- Braitenberg reflex + subsumption command mux
- Automatic telemetry logging, summary generation, and plots
- Manual RViz goal mode and fully automatic benchmark mode

---

## Cognitive and Software Architecture

The overall design is a **hybrid reactive-deliberative cognitive architecture**:

- The hierarchical path performs goal interpretation, global planning, local control, execution monitoring, replanning, and recovery.
- The reflexive path maps immediate obstacle information to a short-term safe velocity command.
- The subsumption mux passes the normal Nav2 command when the reflex is inactive and overrides it with the Braitenberg command when the reflex is active.

<p align="center">
  <img src="docs/images/architecture.png" alt="Hybrid reactive-deliberative architecture" width="100%">
</p>


---

## Benchmark Scenarios

<p align="center">
  <img src="docs/images/scenarios.png" alt="Indoor benchmark scenarios" width="90%">
</p>

| Scenario ID | Purpose | Main challenge |
|---|---|---|
| `corridor` | Baseline traversal | Straight open navigation and timing |
| `doorway` | Narrow passage | Footprint, alignment, and final turning |
| `clutter` | Static obstacle adaptation | Local trajectory selection and path smoothness |
| `house_dynamic` | Dynamic rerouting | A door-like obstacle blocks the original route and an alternative door opens |
| `maze` | Complex route following | Sharp turns, high curvature, and constrained geometry |

Additional scenarios available in the package include `blockage`, `house`, and `multi_goal`.

---

## Evaluated Planner Profiles

| Planner ID | Nav2 profile | Role |
|---|---|---|
| `dwb` | DWB | Conservative baseline |
| `mppi` | MPPI | Predictive sampled controller |
| `rpp` | Regulated Pure Pursuit | Robust and conservative path tracking |
| `teb` | External TEB profile | Optional; requires a compatible ROS 2 Humble Nav2 plugin |

All reported experiments used the same maps, start/goal poses, robot footprint, velocity limits, costmaps, goal tolerance, timeout, and ThetaStar global planner. Only the local planner/controller profile changed.

---

## Benchmark Summary

The final report used 5 scenarios x 3 planners x 3 repetitions = **45 runs**.

<p align="center">
  <img src="docs/images/success_rate.png" alt="Navigation success rate" width="95%">
</p>

<p align="center">
  <img src="docs/images/completion_time.png" alt="Mean completion time" width="95%">
</p>

| Planner | Overall success | Main observation |
|---|---:|---|
| DWB | 60% | Completed the easier tasks but lost progress in difficult sharp-turn/high-curvature sections |
| MPPI | 100% | Best overall combination of reliability, completion time, and final goal accuracy |
| RPP | 100% | Reliable and safe, but generally slower and slightly less precise than MPPI |

In `house_dynamic`, the Braitenberg-inspired reflex handled the immediate obstacle cue, while global replanning and the local planner/controller handled the longer-term route change. Nav2 recovery remained a separate mechanism used when progress was lost.

---

## Requirements

Recommended environment:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic with `gazebo_ros`
- Nav2 and RViz2
- `g1_description_ros2`
- `g1_gazebo`
- Python 3 with NumPy, pandas, Matplotlib, and PyYAML

The package dependencies are declared in `package.xml`. After cloning, install missing ROS dependencies with `rosdep`.

---

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository/package here
# git clone <your-repository-url> g1_cogar_nav_benchmark

cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add the workspace setup to every new terminal before running the project:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## Clean Previous Gazebo/ROS Processes

Use this before restarting a failed or interrupted run.

<details>
<summary>Cleanup commands</summary>

```bash
cd ~/ros2_ws

pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f gazebo 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f benchmark_runner 2>/dev/null || true
pkill -9 -f benchmark_logger 2>/dev/null || true
pkill -9 -f house_dynamic_obstacle_commander 2>/dev/null || true
pkill -9 -f k3_nav_bringup.launch.py 2>/dev/null || true
pkill -9 -f controller_server 2>/dev/null || true
pkill -9 -f planner_server 2>/dev/null || true
pkill -9 -f bt_navigator 2>/dev/null || true
pkill -9 -f behavior_server 2>/dev/null || true
pkill -9 -f lifecycle_manager 2>/dev/null || true
pkill -9 -f map_server 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f spawn_entity.py 2>/dev/null || true

rm -rf /tmp/gazebo-* 2>/dev/null || true
rm -rf /tmp/ignition-* 2>/dev/null || true
rm -rf /tmp/ros2_* 2>/dev/null || true
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

ros2 daemon stop 2>/dev/null || true
sleep 3
ros2 daemon start 2>/dev/null || true
sleep 3
```

</details>

---

## Running the Project

### 1. Automatic Single Run

`run_one.sh` launches the selected world and Nav2 profile, reads the start and goal from `benchmark_scenarios.yaml`, sends the goal automatically, records telemetry, and creates a run summary.

Example: corridor with RPP and GUI enabled.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

USE_GAZEBO_GUI=true \
USE_RVIZ=true \
STARTUP_WAIT=60 \
GOAL_TIMEOUT=265 \
bash src/g1_cogar_nav_benchmark/scripts/run_one.sh \
  corridor \
  rpp \
  results_video/corridor_rpp_demo
```

General form:

```bash
bash src/g1_cogar_nav_benchmark/scripts/run_one.sh \
  <scenario_id> \
  <planner_id> \
  <result_directory>
```

Example:

```bash
USE_GAZEBO_GUI=true USE_RVIZ=true \
  bash src/g1_cogar_nav_benchmark/scripts/run_one.sh \
  house_dynamic mppi results/manual_tests/house_dynamic_mppi
```

### 2. Manual Goal Selection in RViz

This mode launches the simulation and Nav2 but does not send the navigation goal automatically. Use the **Nav2 Goal** tool in RViz to choose the destination interactively.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch g1_cogar_nav_benchmark k3_nav_bringup.launch.py \
  scenario_id:=house_dynamic \
  planner_id:=dwb \
  use_rviz:=true \
  use_gazebo_gui:=true \
  use_reflex:=true \
  logger_output_file:=$PWD/manual_house_dynamic_log.csv
```

After the stack is ready:

1. Wait for Gazebo, Nav2, and RViz to finish starting.
2. In RViz, select **Nav2 Goal** (or **2D Goal Pose**, depending on the RViz toolbar configuration).
3. Click and drag on the map to define the target pose.
4. Observe `/plan`, local costmaps, robot motion, and the reflex/mux topics.

The robot start pose is loaded from the scenario configuration and simulator baseline localization is used.

### 3. Full Benchmark Batch

The following command reproduces the reported 45-run matrix.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

SCENARIOS="corridor doorway clutter house_dynamic maze" \
PLANNERS="dwb mppi rpp" \
RUNS=3 \
USE_RVIZ=false \
USE_GAZEBO_GUI=false \
STARTUP_WAIT=45 \
GOAL_TIMEOUT=240 \
bash src/g1_cogar_nav_benchmark/scripts/run_all_benchmarks.sh
```

The script runs each scenario/planner combination, cleans Gazebo and ROS processes between runs, aggregates all CSV summaries, and generates plots.

---

## Braitenberg Reflex and Subsumption

The reflex node subscribes to `/scan`, measures front/left/right obstacle proximity, and publishes:

- `/reflex_cmd_vel` - short-term safe command
- `/reflex_active` - reflex activation state
- `/reflex_min_range` - nearest measured obstacle distance

The subsumption mux subscribes to:

- `/nav_cmd_vel` - normal Nav2 command
- `/reflex_cmd_vel` - reflex command
- `/reflex_active` - arbitration state

It publishes the selected final command to `/cmd_vel` and the active mode to `/control_mode`.

To enable the reflex in a direct launch, pass:

```bash
use_reflex:=true
```

> Note: the current `run_one.sh` helper uses the launch-file default for `use_reflex`. If automated runs must always include the reflex, expose `use_reflex` as a helper-script argument or change the launch default deliberately.

Useful checks:

```bash
ros2 topic echo /reflex_active
ros2 topic echo /control_mode
ros2 topic hz /scan
ros2 topic echo /nav_cmd_vel
ros2 topic echo /cmd_vel
```

---

## Output Files

A single automated run creates a directory similar to:

```text
results/raw_runs/<scenario>/<run>/<planner>_.../
├── benchmark_log.csv
├── telemetry_live.csv
├── run_metadata.yaml
├── stack.log
└── summary.csv
```

A full benchmark batch also creates:

```text
results/
├── raw_runs/
├── summary/
│   ├── all_runs.csv
│   └── summary.csv
└── plots/
```

Primary metrics include:

- success rate
- completion time
- path efficiency
- final goal error
- minimum obstacle clearance
- collision-proxy events
- Nav2 recovery count
- Braitenberg reflex activation

---

## Repository Structure

```text
g1_cogar_nav_benchmark/
├── config/          # Scenario, Nav2, planner, and behavior-tree configuration
├── launch/          # Main Gazebo + Nav2 launch files
├── maps/            # Known maps and map metadata
├── worlds/          # Gazebo benchmark worlds
├── rviz/            # RViz configuration
├── scripts/         # Single-run, batch-run, and dynamic-obstacle scripts
├── src/             # C++ nodes: runner, logger, reflex, mux, goal manager
├── tools/           # Aggregation, summarization, and plotting utilities
├── include/         # Shared C++ headers
└── g1_cogar_nav_benchmark/  # Python helper modules and simplified G1 motion
```

---

## Limitations

- The evaluation is simulation-only and uses simulator-provided baseline localization.
- Each planner/scenario pair was repeated three times.
- The humanoid locomotion model is intentionally simplified. High-fidelity motion for a 29-DOF humanoid would require whole-body dynamics, gait generation, balance control, and substantially more computation.
- The collision metric is proximity-based and is not equivalent to physical-contact ground truth.
- The available recovery sequence did not resolve every DWB local-minimum failure.

---

## Future Work

- Real Unitree G1 experiments
- Moving-human and richer dynamic-obstacle scenarios
- Failure-aware escape recovery with collision-checked translation or backup
- Parameter sensitivity and automated tuning
- Higher-fidelity full-body dynamics and human-like gait simulation
- More repetitions and statistical analysis

---

## Main Finding

For the tested simulation configuration:

- **MPPI** is the recommended overall profile.
- **RPP** is a robust and conservative alternative.
- **DWB** remains a useful baseline and exposes failure cases involving sharp turns, high curvature, and ineffective recovery from local minima.

---

## Author

**Mahdi Baghban Ghalehchi**  
Cognitive Architectures for Robotics - COGAR K3 Project  

