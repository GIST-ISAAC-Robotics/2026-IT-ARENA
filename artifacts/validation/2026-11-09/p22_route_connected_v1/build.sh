#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-09/p22_route_connected_v1
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
colcon build --symlink-install --packages-select arena_autonomy arena_bringup >"$A/build.log" 2>&1
source install/local_setup.bash
ros2 pkg executables arena_autonomy >"$A/executables.txt"
grep -q 'arena_autonomy p22_route_trial' "$A/executables.txt"
