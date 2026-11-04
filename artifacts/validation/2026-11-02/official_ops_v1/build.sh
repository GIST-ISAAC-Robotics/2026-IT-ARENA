#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$A/build.log" 2>&1
echo "BUILD=$?"
source install/local_setup.bash
ros2 pkg executables arena_autonomy | grep race_ops
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_race_recovery.py tests/test_race_ops.py tests/test_race_session.py tests/test_race_session_node.py tests/test_race_operator.py tests/test_race_ops_node.py 2>&1 | tail -5
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_standing_wall_reacquire.py tests/test_low_speed_arc_reacquire.py tests/test_boundary_bridge.py 2>&1 | tail -12
