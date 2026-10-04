#!/usr/bin/env bash
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
outdir='artifacts/validation/2026-10-05/g0_unit_v3'
mkdir -p "$outdir"

echo '=== ROS interface ==='
ros2 interface show ros_gz_interfaces/srv/SetEntityPose > "$outdir/set_entity_pose_interface.log" 2>&1
printf '%s\n' "$?" > "$outdir/set_entity_pose_interface.returncode"
cat "$outdir/set_entity_pose_interface.log"

echo '=== ros_gz_interfaces prefix ==='
ros2 pkg prefix ros_gz_interfaces > "$outdir/ros_gz_interfaces_prefix.log" 2>&1
printf '%s\n' "$?" > "$outdir/ros_gz_interfaces_prefix.returncode"
cat "$outdir/ros_gz_interfaces_prefix.log"

echo '=== ros_gz_bridge executable ==='
command -v ros_gz_bridge > "$outdir/ros_gz_bridge_command.log" 2>&1
printf '%s\n' "$?" > "$outdir/ros_gz_bridge_command.returncode"
cat "$outdir/ros_gz_bridge_command.log"

echo '=== ros_gz_bridge help (if available) ==='
if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  ros2 run ros_gz_bridge parameter_bridge --help > "$outdir/ros_gz_bridge_help.log" 2>&1
  printf '%s\n' "$?" > "$outdir/ros_gz_bridge_help.returncode"
  grep -in -C 2 -E 'service|SetEntityPose|request|response' "$outdir/ros_gz_bridge_help.log" || true
else
  printf 'ros_gz_bridge package unavailable\n' > "$outdir/ros_gz_bridge_help.log"
  printf '127\n' > "$outdir/ros_gz_bridge_help.returncode"
  cat "$outdir/ros_gz_bridge_help.log"
fi

echo '=== nine-file pytest regression ==='
/usr/bin/python3 -m pytest -q \
  tests/test_local_scene.py \
  tests/test_g0_geometry.py \
  tests/test_multi_vehicle_g0.py \
  tests/test_r0_safety_contract.py \
  tests/test_local_pursuit.py \
  tests/test_lidar_safety.py \
  tests/test_lidar_observation.py \
  tests/test_lidar_motion.py \
  tests/test_rotating_lidar.py \
  > "$outdir/pytest.log" 2>&1
pytest_rc=$?
printf '%s\n' "$pytest_rc" > "$outdir/pytest.returncode"
cat "$outdir/pytest.log"
printf 'pytest_returncode=%s\n' "$pytest_rc"
exit "$pytest_rc"
