#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_positive_clearance_v1
python3 -B -m pytest -q -p no:cacheprovider tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_jetson_bundle_dependencies.py tests/test_source_provenance_r0_plus_b3.py tests/test_anchor_set_provenance.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_corridor_roles.py tests/test_physics_step_option.py > "$base/bundle_static.log" 2>&1
colcon build --packages-select arena_autonomy --symlink-install --event-handlers console_direct+ > "$base/build.log" 2>&1
python3 -B "$base/run1_snapshot.py" > "$base/run1_snapshot.log" 2>&1
bash "$base/run1_trial.sh"
python3 -B "$base/summarize_run.py" > "$base/summarize_run.log" 2>&1
