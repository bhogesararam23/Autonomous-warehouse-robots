#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_root="$repo_root/ros2_ws/src/warehouse_astar_planner"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

cxx="${CXX:-g++}"
"$cxx" \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Werror \
  -O2 \
  -I "$package_root/include" \
  "$package_root/src/grid_astar.cpp" \
  "$package_root/test/grid_astar_test.cpp" \
  -o "$build_dir/grid_astar_test"

"$build_dir/grid_astar_test"
