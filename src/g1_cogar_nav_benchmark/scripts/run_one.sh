#!/usr/bin/env bash
set -euo pipefail

SCENARIO_ID="${1:-doorway}"
PLANNER_ID="${2:-dwb}"
RESULT_DIR="${3:-$PWD/results/raw_runs/${PLANNER_ID}_${SCENARIO_ID}_run01}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_GAZEBO_GUI="${USE_GAZEBO_GUI:-false}"
STARTUP_WAIT="${STARTUP_WAIT:-10.0}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-240.0}"
TEB_PLUGIN_CLASS="${TEB_PLUGIN_CLASS:-}"

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

if [[ "${PLANNER_ID}" == "teb" ]]; then
  if [[ "${TEB_NAV2_PLUGIN_AVAILABLE:-0}" != "1" || -z "${TEB_PLUGIN_CLASS}" ]]; then
    echo "[ERROR] Planner 'teb' requires an external ROS2/Humble-compatible TEB controller plugin."
    echo "        Export TEB_NAV2_PLUGIN_AVAILABLE=1 and TEB_PLUGIN_CLASS='<plugin class>' only if that plugin is installed."
    exit 3
  fi
fi

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

SCENARIO_FILE="${PKG_SHARE}/config/benchmark_scenarios.yaml"
LOGGER_OUTPUT_FILE="${RESULT_DIR}/telemetry_live.csv"
LAUNCH_LOG="${RESULT_DIR}/stack.log"
RUN_ID="$(basename "${RESULT_DIR}")"

mkdir -p "${RESULT_DIR}"
source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WORKSPACE_PREFIX}/setup.bash"

cleanup() {
  if [[ -n "${STACK_PID:-}" ]]; then
    kill "${STACK_PID}" >/dev/null 2>&1 || true
    wait "${STACK_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "[INFO] Launching stack for scenario=${SCENARIO_ID}, planner=${PLANNER_ID}, run=${RUN_ID}"
LAUNCH_ARGS=(
  "scenario_file:=${SCENARIO_FILE}"
  "scenario_id:=${SCENARIO_ID}"
  "planner_id:=${PLANNER_ID}"
  "run_id:=${RUN_ID}"
  "use_rviz:=${USE_RVIZ}"
  "use_gazebo_gui:=${USE_GAZEBO_GUI}"
  "logger_output_file:=${LOGGER_OUTPUT_FILE}"
)
if [[ -n "${TEB_PLUGIN_CLASS}" ]]; then
  LAUNCH_ARGS+=("teb_plugin_class:=${TEB_PLUGIN_CLASS}")
fi

ros2 launch g1_cogar_nav_benchmark k3_nav_bringup.launch.py "${LAUNCH_ARGS[@]}" >"${LAUNCH_LOG}" 2>&1 &
STACK_PID=$!

READY=0
for _ in $(seq 1 150); do
  if ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$'; then
    READY=1
    break
  fi
  sleep 1
  if ! kill -0 "${STACK_PID}" 2>/dev/null; then
    echo "[ERROR] Launch stack terminated early. Check ${LAUNCH_LOG}"
    tail -n 50 "${LAUNCH_LOG}" || true
    exit 4
  fi
done

if [[ "${READY}" != "1" ]]; then
  echo "[ERROR] Nav2 action server did not become available. Check ${LAUNCH_LOG}"
  tail -n 50 "${LAUNCH_LOG}" || true
  exit 5
fi

set +e
ros2 run g1_cogar_nav_benchmark benchmark_runner \
  --scenario-file "${SCENARIO_FILE}" \
  --scenario-id "${SCENARIO_ID}" \
  --planner-id "${PLANNER_ID}" \
  --result-dir "${RESULT_DIR}" \
  --startup-wait "${STARTUP_WAIT}" \
  --goal-timeout "${GOAL_TIMEOUT}" \
  --logger-output-file "${LOGGER_OUTPUT_FILE}"
RUN_RC=$?
set -e

if [[ ${RUN_RC} -ne 0 && ${RUN_RC} -ne 2 ]]; then
  echo "[ERROR] benchmark_runner failed with code ${RUN_RC}. Check ${LAUNCH_LOG}"
  exit ${RUN_RC}
fi

read -r GOAL_X GOAL_Y <<<"$(python3 -c 'import sys, yaml; cfg=yaml.safe_load(open(sys.argv[1])); g=cfg["scenarios"][sys.argv[2]]["goal_pose"]; print(g[0], g[1])' "${SCENARIO_FILE}" "${SCENARIO_ID}")"
SUCCESS_FLAG="0"
if [[ -f "${RESULT_DIR}/run_metadata.yaml" ]]; then
  SUCCESS_FLAG="$(grep -E '^success:' "${RESULT_DIR}/run_metadata.yaml" | awk '{print $2}')"
fi

python3 "${TOOL_DIR}/summarize_results.py" \
  --log-csv "${RESULT_DIR}/benchmark_log.csv" \
  --scenario-id "${SCENARIO_ID}" \
  --planner-id "${PLANNER_ID}" \
  --run-id "${RUN_ID}" \
  --goal-x "${GOAL_X}" \
  --goal-y "${GOAL_Y}" \
  --success "${SUCCESS_FLAG}" \
  --scenario-file "${SCENARIO_FILE}" \
  --metadata-yaml "${RESULT_DIR}/run_metadata.yaml" \
  --output-csv "${RESULT_DIR}/summary.csv"

echo "[INFO] Run completed with benchmark_runner rc=${RUN_RC}. Results in ${RESULT_DIR}"
exit 0
