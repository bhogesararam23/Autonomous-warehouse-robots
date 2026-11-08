#!/usr/bin/env bash
# Reproducible, simulation-only NavigateToPose smoke test for the A300 A* plugin.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="/opt/ros/jazzy/setup.bash"
workspace_setup="$repo_root/ros2_ws/install/setup.bash"
profile_source="$repo_root/ros2_ws/src/warehouse_astar_planner/config/robot.yaml"
helper="$repo_root/scripts/prepare_husky_nav2_smoke.py"
headless_launch="$repo_root/scripts/husky_nav2_headless.launch.py"
smoke_client="$repo_root/scripts/husky_nav2_e2e_smoke.py"
run_root="${HUSKY_SMOKE_RUN_ROOT:-${TMPDIR:-/tmp}}"
run_dir="$(mktemp -d "$run_root/husky-nav2-e2e.XXXXXX")"
setup_dir="$run_dir/clearpath"
world_file="$run_dir/warehouse_contacts.sdf"
manifest="$run_dir/contact_manifest.json"
fault_model="$run_dir/blocked_aisle_obstacle.sdf"
fault_injection="${HUSKY_SMOKE_FAULT:-none}"
smoke_fault_args=()
smoke_fault_model_args=()
default_action_timeout=180
case "$fault_injection" in
  none) ;;
  blocked-aisle)
    # Hosted software rendering makes the full route-around slower than direct goals.
    default_action_timeout=600
    smoke_fault_args=(
      --fault-injection blocked-aisle
      --minimum-goal-distance-m 1.8
    )
    smoke_fault_model_args=(--fault-model-file "$fault_model")
    ;;
  *)
    echo "Unsupported HUSKY_SMOKE_FAULT: $fault_injection" >&2
    exit 2
    ;;
esac
smoke_action_timeout="${HUSKY_SMOKE_ACTION_TIMEOUT:-$default_action_timeout}"
mkdir -p "$setup_dir"

sim_pid=""
slam_pid=""
nav2_pid=""

stop_one() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 25); do
      kill -0 -- "-$pid" 2>/dev/null || break
      sleep 0.4
    done
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      for _ in $(seq 1 10); do
        kill -0 -- "-$pid" 2>/dev/null || break
        sleep 0.2
      done
    fi
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  stop_one "$nav2_pid"
  stop_one "$slam_pid"
  stop_one "$sim_pid"
  if [[ "$exit_code" -ne 0 ]]; then
    echo "Smoke test failed. Logs and generated evidence are preserved in: $run_dir" >&2
    for log in prepare.log simulation.log slam.log nav2.log smoke.log; do
      if [[ -s "$run_dir/$log" ]]; then
        echo "--- tail: $log ---" >&2
        tail -n 30 "$run_dir/$log" >&2 || true
      fi
    done
  else
    echo "Smoke-test artifacts: $run_dir"
  fi
  exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Jazzy setup is missing: $ros_setup" >&2
  exit 2
fi
if [[ ! -f "$workspace_setup" ]]; then
  echo "ROS workspace is not built: $workspace_setup (see docs/HUSKY_A300_SIMULATION.md)" >&2
  exit 2
fi

# ROS/Clearpath generator entry points use /usr/bin/env python3; the distro
# interpreter carries the apt-installed catkin_pkg and PyYAML modules.
export PATH="/usr/bin:$PATH"
source "$ros_setup"
for executable in ros2 gz xacro python3 setsid; do
  command -v "$executable" >/dev/null 2>&1 || {
    echo "Required executable is missing: $executable" >&2
    exit 2
  }
done
cp "$profile_source" "$setup_dir/robot.yaml"
ros2 run clearpath_generator_common generate_bash -s "$setup_dir" \
  >"$run_dir/prepare.log" 2>&1
source "$setup_dir/setup.bash"
ros2 run clearpath_generator_common generate_description -s "$setup_dir" \
  >>"$run_dir/prepare.log" 2>&1
ros2 run clearpath_generator_common generate_semantic_description -s "$setup_dir" \
  >>"$run_dir/prepare.log" 2>&1
ros2 run clearpath_generator_gz generate_launch -s "$setup_dir" \
  >>"$run_dir/prepare.log" 2>&1
ros2 run clearpath_generator_gz generate_param -s "$setup_dir" \
  >>"$run_dir/prepare.log" 2>&1
source "$workspace_setup"
# Clearpath's temporary profile defaults to domain 0; isolate the test graph.
export ROS_DOMAIN_ID="${HUSKY_SMOKE_DOMAIN_ID:-57}"
# Keep Gazebo Transport discovery private to this invocation as well.
export GZ_PARTITION="${HUSKY_SMOKE_GZ_PARTITION:-husky-smoke-${ROS_DOMAIN_ID}-$$}"

clearpath_gz_share="$(ros2 pkg prefix clearpath_gz)/share/clearpath_gz"
python3 "$helper" \
  --setup-path "$setup_dir" \
  --source-world "$clearpath_gz_share/worlds/warehouse.sdf" \
  --output-world "$world_file" \
  --manifest "$manifest" \
  --fault-model-output "$fault_model" \
  --fault-injection "$fault_injection" \
  >>"$run_dir/prepare.log" 2>&1
contact_count="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["contact_sensor_count"])' "$manifest")"

printf 'Run directory: %s\nInstrumented robot collision geometries: %s\n' "$run_dir" "$contact_count"

setsid ros2 launch "$headless_launch" \
  setup_path:="$setup_dir" \
  world_path:="$world_file" \
  contact_count:="$contact_count" \
  x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 \
  >"$run_dir/simulation.log" 2>&1 &
sim_pid=$!

wait_for_publisher() {
  local topic="$1"
  local limit_seconds="$2"
  local elapsed=0
  while (( elapsed < limit_seconds )); do
    local info
    info="$(timeout 4s ros2 topic info "$topic" 2>/dev/null || true)"
    if grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$info"; then
      echo "Ready: $topic"
      return 0
    fi
    if ! kill -0 "$sim_pid" 2>/dev/null; then
      echo "Simulation process stopped before $topic appeared" >&2
      return 1
    fi
    sleep 1
    ((elapsed += 1))
  done
  echo "Timed out after ${limit_seconds}s waiting for a publisher on $topic" >&2
  return 1
}

wait_for_publisher /a300_00000/sensors/lidar2d_0/scan 150

setsid ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:="$setup_dir" use_sim_time:=true \
  >"$run_dir/slam.log" 2>&1 &
slam_pid=$!
wait_for_publisher /a300_00000/map 150

setsid ros2 launch warehouse_astar_planner nav2.launch.py \
  setup_path:="$setup_dir" use_sim_time:=true \
  >"$run_dir/nav2.log" 2>&1 &
nav2_pid=$!

set +e
python3 "$smoke_client" \
  --contact-count "$contact_count" \
  --result-file "$run_dir/result.json" \
  --fault-manifest "$manifest" \
  --world-file "$world_file" \
  --action-timeout "$smoke_action_timeout" \
  "${smoke_fault_model_args[@]}" \
  "$@" \
  "${smoke_fault_args[@]}" \
  > >(tee "$run_dir/smoke.log") 2>&1
smoke_status=$?
set -e
if [[ "$smoke_status" -ne 0 ]]; then
  exit "$smoke_status"
fi

echo "PASS: end-to-end simulation-only navigation and collision smoke test."
