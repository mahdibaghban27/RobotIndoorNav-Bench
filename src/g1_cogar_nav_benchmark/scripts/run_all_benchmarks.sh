#!/usr/bin/env bash
set -euo pipefail

SCENARIOS=(${SCENARIOS:-corridor doorway clutter house_dynamic maze})
PLANNERS=(${PLANNERS:-dwb mppi rpp})
if [[ "${INCLUDE_TEB:-0}" == "1" ]]; then
  PLANNERS+=(teb)
fi

RUNS="${RUNS:-3}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_GAZEBO_GUI="${USE_GAZEBO_GUI:-false}"
STARTUP_WAIT="${STARTUP_WAIT:-45.0}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-240.0}"
TEB_PLUGIN_CLASS="${TEB_PLUGIN_CLASS:-}"
TEB_NAV2_PLUGIN_AVAILABLE="${TEB_NAV2_PLUGIN_AVAILABLE:-0}"
BASE_RESULTS_DIR="${BASE_RESULTS_DIR:-$PWD/results}"
BATCH_ID="${BATCH_ID:-$(date +%Y%m%d_%H%M%S)}"

cleanup_sim()
{
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f spawn_entity.py 2>/dev/null || true
  pkill -9 -f benchmark_runner 2>/dev/null || true
  pkill -9 -f benchmark_logger 2>/dev/null || true
  pkill -9 -f fake_walk_animator.py 2>/dev/null || true
  pkill -9 -f k3_nav_bringup.launch.py 2>/dev/null || true
  pkill -9 -f house_dynamic_obstacle_commander 2>/dev/null || true
  pkill -9 -f dynamic_obstacle_commander 2>/dev/null || true
  pkill -9 -f controller_server 2>/dev/null || true
  pkill -9 -f planner_server 2>/dev/null || true
  pkill -9 -f bt_navigator 2>/dev/null || true
  pkill -9 -f nav2_recoveries 2>/dev/null || true
  pkill -9 -f behavior_server 2>/dev/null || true
  pkill -9 -f velocity_smoother 2>/dev/null || true
  pkill -9 -f waypoint_follower 2>/dev/null || true
  pkill -9 -f lifecycle_manager 2>/dev/null || true
  pkill -9 -f map_server 2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f static_transform_publisher 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
  pkill -9 -f braitenberg_reflex 2>/dev/null || true
  pkill -9 -f subsumption_mux 2>/dev/null || true
  pkill -9 -f rviz2 2>/dev/null || true
  # Wait for gzserver to fully release memory before next run
  local waited=0
  while pgrep -x gzserver >/dev/null 2>&1 && [[ ${waited} -lt 15 ]]; do
    sleep 1; waited=$((waited + 1))
  done
  ros2 daemon stop 2>/dev/null || true
  sleep 20
  ros2 daemon start 2>/dev/null || true
  sleep 5
}

source_setup_file() {
  local setup_file="$1"
  [[ -f "${setup_file}" ]] || return 0
  local had_nounset=0
  case "$-" in
    *u*) had_nounset=1 ;;
  esac
  set +u
  source "${setup_file}"
  if [[ "${had_nounset}" == "1" ]]; then
    set -u
  fi
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "${SCRIPT_DIR}/../config" ]]; then
  PKG_SHARE="$(cd "${SCRIPT_DIR}/.." && pwd)"
  PKG_LIB="${SCRIPT_DIR}"
  TOOL_DIR="${PKG_SHARE}/tools"
  WORKSPACE_PREFIX="$(cd "${SCRIPT_DIR}/../../.." && pwd)/install"
else
  PKG_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)"
  WORKSPACE_PREFIX="$(dirname "${PKG_PREFIX}")"
  PKG_SHARE="${PKG_PREFIX}/share/g1_cogar_nav_benchmark"
  PKG_LIB="${PKG_PREFIX}/lib/g1_cogar_nav_benchmark"
  TOOL_DIR="${PKG_LIB}"
fi

RUN_ONE="${PKG_LIB}/run_one.sh"
AGGREGATE="${TOOL_DIR}/aggregate_results.py"
PLOT="${TOOL_DIR}/plot_results.py"
RAW_DIR="${BASE_RESULTS_DIR}/raw_runs"
SUMMARY_DIR="${BASE_RESULTS_DIR}/summary"
PLOTS_DIR="${BASE_RESULTS_DIR}/plots"

mkdir -p "${RAW_DIR}" "${SUMMARY_DIR}" "${PLOTS_DIR}"
source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WORKSPACE_PREFIX}/setup.bash"

cleanup_sim

echo "[INFO] Batch id: ${BATCH_ID}"
echo "[INFO] Run order: scenario -> run -> planner"
echo "[INFO] Scenarios: ${SCENARIOS[*]}"
echo "[INFO] Planners:  ${PLANNERS[*]}"
echo "[INFO] Runs per scenario/planner: ${RUNS}"
echo "[INFO] STARTUP_WAIT=${STARTUP_WAIT} GOAL_TIMEOUT=${GOAL_TIMEOUT}"
echo "[INFO] USE_RVIZ=${USE_RVIZ} USE_GAZEBO_GUI=${USE_GAZEBO_GUI}"

for scenario in "${SCENARIOS[@]}"; do
  for run_no in $(seq 1 "${RUNS}"); do
    printf -v run_tag "run_%02d" "${run_no}"
    group_dir="${RAW_DIR}/${scenario}/${run_tag}"

    for planner in "${PLANNERS[@]}"; do
      result_dir="${group_dir}/${planner}_${scenario}_${run_tag}_${BATCH_ID}"

      cleanup_sim

      echo "[INFO] Run scenario=${scenario} ${run_tag}/${RUNS} planner=${planner}"

      set +e
      USE_RVIZ="${USE_RVIZ}" \
      USE_GAZEBO_GUI="${USE_GAZEBO_GUI}" \
      STARTUP_WAIT="${STARTUP_WAIT}" \
      GOAL_TIMEOUT="${GOAL_TIMEOUT}" \
      TEB_PLUGIN_CLASS="${TEB_PLUGIN_CLASS}" \
      TEB_NAV2_PLUGIN_AVAILABLE="${TEB_NAV2_PLUGIN_AVAILABLE}" \
        bash "${RUN_ONE}" "${scenario}" "${planner}" "${result_dir}"
      run_rc=$?
      set -e

      cleanup_sim

      if [[ "${run_rc}" -ne 0 ]]; then
        echo "[WARN] Run failed with rc=${run_rc}: planner=${planner} scenario=${scenario} ${run_tag}"
      fi
      cleanup_sim
    done
  done
done

mapfile -t SUMMARY_CSVS < <(find "${RAW_DIR}" -path "*_${BATCH_ID}/summary.csv" | sort)
if [[ ${#SUMMARY_CSVS[@]} -eq 0 ]]; then
  echo "[ERROR] No summary.csv files were generated."
  exit 6
fi

python3 "${AGGREGATE}" \
  --inputs "${SUMMARY_CSVS[@]}" \
  --output "${SUMMARY_DIR}/summary.csv" \
  --raw-output "${SUMMARY_DIR}/all_runs.csv"

python3 "${PLOT}" --summary "${SUMMARY_DIR}/summary.csv" --output-dir "${PLOTS_DIR}"

echo "[INFO] Raw runs: ${RAW_DIR}"
echo "[INFO] Summary:  ${SUMMARY_DIR}/summary.csv"
echo "[INFO] Plots:    ${PLOTS_DIR}"
