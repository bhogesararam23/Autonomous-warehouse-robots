#!/usr/bin/env bash
# Simulation-only Open-RMF task dispatch / traffic scheduling smoke for two A300s.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="/opt/ros/jazzy/setup.bash"
workspace_setup="$repo_root/ros2_ws/install/setup.bash"
profile_source="$repo_root/ros2_ws/src/warehouse_astar_planner/config/robot.yaml"
profile_generator="$repo_root/scripts/prepare_a300_fleet_profiles.py"
instrumenter="$repo_root/scripts/prepare_husky_nav2_smoke.py"
smoke_client="$repo_root/scripts/rmf_a300_fleet_smoke.py"
launch_name="warehouse_rmf_fleet a300_fleet.launch.py"
run_root="${HUSKY_RMF_FLEET_RUN_ROOT:-${TMPDIR:-/tmp}}"
run_dir="$(mktemp -d "$run_root/husky-rmf-fleet.XXXXXX")"
profiles_root="$run_dir/profiles"
world_file="$run_dir/warehouse_contacts.sdf"
result_file="$run_dir/fleet_result.json"
mkdir -p "$profiles_root"

launch_pid=""
stop_launch() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
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
  stop_launch "$launch_pid"
  if [[ "$exit_code" -ne 0 ]]; then
    echo "Fleet smoke test failed; logs and evidence are preserved in: $run_dir" >&2
    for log in prepare.log simulation.log smoke.log; do
      if [[ -s "$run_dir/$log" ]]; then
        echo "--- tail: $log ---" >&2
        tail -n 35 "$run_dir/$log" >&2 || true
      fi
    done
  else
    echo "Fleet smoke-test artifacts: $run_dir"
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
  echo "ROS workspace is not built: $workspace_setup (build ros2_ws first)" >&2
  exit 2
fi
export PATH="/usr/bin:$PATH"
source "$ros_setup"
for executable in ros2 gz xacro python3 setsid; do
  command -v "$executable" >/dev/null 2>&1 || {
    echo "Required executable is missing: $executable" >&2
    exit 2
  }
done

python3 "$profile_generator" --template "$profile_source" --output-dir "$profiles_root" \
  >"$run_dir/prepare.log" 2>&1
for robot in a300_00000 a300_00001; do
  setup_path="$profiles_root/$robot"
  ros2 run clearpath_generator_common generate_bash -s "$setup_path" \
    >>"$run_dir/prepare.log" 2>&1
  source "$setup_path/setup.bash"
  ros2 run clearpath_generator_common generate_description -s "$setup_path" \
    >>"$run_dir/prepare.log" 2>&1
  ros2 run clearpath_generator_common generate_semantic_description -s "$setup_path" \
    >>"$run_dir/prepare.log" 2>&1
  ros2 run clearpath_generator_gz generate_launch -s "$setup_path" \
    >>"$run_dir/prepare.log" 2>&1
  ros2 run clearpath_generator_gz generate_param -s "$setup_path" \
    >>"$run_dir/prepare.log" 2>&1
  python3 - "$setup_path" "$robot" <<'PY' >>"$run_dir/prepare.log" 2>&1
import sys
from pathlib import Path
import yaml

setup_path = Path(sys.argv[1])
namespace = sys.argv[2]
control_path = setup_path / "platform/config/control.yaml"
control = yaml.safe_load(control_path.read_text(encoding="utf-8"))
parameters = control[namespace]["platform_velocity_controller"]["ros__parameters"]
parameters["enable_odom_tf"] = True
control_path.write_text(yaml.safe_dump(control, sort_keys=False), encoding="utf-8")
print(f"Enabled simulated differential-drive odometry TF for {namespace}")
PY
done
source "$workspace_setup"
export ROS_DOMAIN_ID="${HUSKY_RMF_FLEET_DOMAIN_ID:-58}"
export GZ_PARTITION="${HUSKY_RMF_GZ_PARTITION:-husky-rmf-fleet-${ROS_DOMAIN_ID}-$$}"
clearpath_gz_share="$(ros2 pkg prefix clearpath_gz)/share/clearpath_gz"

python3 "$instrumenter" \
  --setup-path "$profiles_root/a300_00000" \
  --source-world "$clearpath_gz_share/worlds/warehouse.sdf" \
  --output-world "$world_file" \
  --manifest "$run_dir/contact_a300_00000.json" \
  --robot-namespace a300_00000 \
  >>"$run_dir/prepare.log" 2>&1
python3 "$instrumenter" \
  --setup-path "$profiles_root/a300_00001" \
  --source-world "$clearpath_gz_share/worlds/warehouse.sdf" \
  --output-world "$world_file" \
  --manifest "$run_dir/contact_a300_00001.json" \
  --robot-namespace a300_00001 \
  --skip-world-instrumentation \
  >>"$run_dir/prepare.log" 2>&1
contact_count="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["contact_sensor_count"])' "$run_dir/contact_a300_00000.json")"
second_count="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["contact_sensor_count"])' "$run_dir/contact_a300_00001.json")"
if [[ "$contact_count" != "$second_count" ]]; then
  echo "A300 contact sensor counts differ: $contact_count vs $second_count" >&2
  exit 2
fi

echo "Run directory: $run_dir"
echo "Instrumented contact geometries per A300: $contact_count"
setsid ros2 launch $launch_name \
  setup_path_0:="$profiles_root/a300_00000" \
  setup_path_1:="$profiles_root/a300_00001" \
  world_path:="$world_file" \
  contact_count:="$contact_count" \
  robot_0_x:=0.0 robot_1_x:=3.5 robot_1_yaw:=3.141592653589793 \
  >"$run_dir/simulation.log" 2>&1 &
launch_pid=$!

wait_for_publisher() {
  local topic="$1" limit_seconds="$2" elapsed=0
  while (( elapsed < limit_seconds )); do
    local info
    info="$(timeout 4s ros2 topic info "$topic" 2>/dev/null || true)"
    if grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$info"; then
      echo "Ready: $topic"
      return 0
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "Launch stopped before $topic became available" >&2
      return 1
    fi
    sleep 1
    ((elapsed += 1))
  done
  echo "Timed out waiting for a publisher on $topic" >&2
  return 1
}

verify_nav2_parameter() {
  local robot="$1" parameter="$2" expected="$3" output="" actual=""
  for _ in $(seq 1 10); do
    output="$(timeout 6s ros2 param get "/$robot/controller_server" "$parameter" 2>/dev/null || true)"
    actual="$(sed -n 's/^Double value is: //p' <<<"$output" | tail -n 1)"
    if [[ -n "$actual" ]] && python3 - "$actual" "$expected" <<'PY'
import math
import sys
try:
    matches = math.isclose(float(sys.argv[1]), float(sys.argv[2]), rel_tol=0.0, abs_tol=1e-9)
except ValueError:
    matches = False
raise SystemExit(0 if matches else 1)
PY
    then
      echo "Verified Nav2 $robot $parameter=$actual"
      return 0
    fi
    sleep 1
  done
  echo "Nav2 parameter mismatch for $robot $parameter: expected $expected, got ${actual:-unavailable}" >&2
  return 1
}

read -r nav2_xy_goal_tolerance nav2_yaw_goal_tolerance < <(
  python3 - "$repo_root/ros2_ws/src/warehouse_astar_planner/config/planner_params.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as source:
    parameters = yaml.safe_load(source)
checker = parameters["controller_server"]["ros__parameters"]["general_goal_checker"]
print(checker["xy_goal_tolerance"], checker["yaw_goal_tolerance"])
PY
)

wait_for_publisher /a300_00000/sensors/lidar2d_0/scan 180
wait_for_publisher /a300_00001/sensors/lidar2d_0/scan 180
wait_for_publisher /a300_00000/map 180
wait_for_publisher /a300_00001/map 180
wait_for_publisher /rmf_traffic/participants 90
wait_for_publisher /fleet_states 180
for robot in a300_00000 a300_00001; do
  verify_nav2_parameter "$robot" general_goal_checker.xy_goal_tolerance "$nav2_xy_goal_tolerance"
  verify_nav2_parameter "$robot" general_goal_checker.yaw_goal_tolerance "$nav2_yaw_goal_tolerance"
done

set +e
python3 "$smoke_client" \
  --contact-count "$contact_count" \
  --nav2-xy-goal-tolerance-m "$nav2_xy_goal_tolerance" \
  --nav2-yaw-goal-tolerance-rad "$nav2_yaw_goal_tolerance" \
  --timeout "${HUSKY_RMF_FLEET_TIMEOUT:-360}" \
  --result-file "$result_file" \
  > >(tee "$run_dir/smoke.log") 2>&1
smoke_status=$?
set -e
if [[ "$smoke_status" -ne 0 ]]; then
  exit "$smoke_status"
fi

echo "PASS: Open-RMF dispatched both simulated A300s through the shared schedule."
echo "Structured evidence: $result_file"
