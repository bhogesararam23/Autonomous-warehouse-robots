#!/usr/bin/env bash
# Build and test the Jazzy workspace, then validate generated Open-RMF fleet assets.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="/opt/ros/jazzy/setup.bash"
workspace_dir="$repo_root/ros2_ws"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Jazzy setup is missing: $ros_setup" >&2
  exit 2
fi

export PATH="/usr/bin:$PATH"
source "$ros_setup"

cd "$workspace_dir"
colcon build --symlink-install --base-paths src --event-handlers console_direct+
source install/setup.bash
colcon test --base-paths src --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose

cd "$repo_root"
./scripts/test_rmf_fleet_assets.sh

echo "PASS: ROS 2 workspace build, package tests, and Open-RMF asset validation."
