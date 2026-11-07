#!/usr/bin/env bash
# Validate the generated navigation graph, RMF config, and pure coordinate transforms.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$repo_root/ros2_ws/install/setup.bash" ]]; then
  echo "Build ros2_ws before testing the RMF fleet assets." >&2
  exit 2
fi
source "$repo_root/ros2_ws/install/setup.bash"
export PATH="/usr/bin:$PATH"
config_dir="$repo_root/ros2_ws/src/warehouse_rmf_fleet/config"
tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/rmf-a300-assets.XXXXXX")"
trap 'rm -rf "$tmp_dir"' EXIT

ros2 run rmf_building_map_tools building_map_generator nav \
  "$config_dir/warehouse_a300_fleet.building.yaml" "$tmp_dir/nav_graphs"
if ! cmp -s "$config_dir/nav_graphs/0.yaml" "$tmp_dir/nav_graphs/0.yaml"; then
  echo "Checked-in RMF graph differs from the official building_map_generator output." >&2
  diff -u "$config_dir/nav_graphs/0.yaml" "$tmp_dir/nav_graphs/0.yaml" >&2 || true
  exit 1
fi
python3 -m unittest discover \
  -s "$repo_root/ros2_ws/src/warehouse_rmf_fleet/test" -v
ros2 run warehouse_rmf_fleet validate_assets \
  --config "$config_dir/rmf_fleet.yaml" \
  --nav-graph "$config_dir/nav_graphs/0.yaml"
echo "PASS: RMF fleet graph, adapter YAML, and coordinate transforms validate."
