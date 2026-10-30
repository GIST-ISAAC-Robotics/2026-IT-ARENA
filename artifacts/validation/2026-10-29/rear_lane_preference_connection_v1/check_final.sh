#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_lane_preference_connection_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/final_tests.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_observed_lane_preference_provenance.py tests/test_rear_lane_keep_provenance.py tests/test_source_contract_helpers.py tests/test_source_provenance_r0_plus_b3.py tests/test_observed_lane_preference_trial.py tests/test_follow_profile.py tests/test_jetson_bundle_dependencies.py > "$base/final_tests.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/final_tests.exit"
[ "$rc" = 0 ] || exit "$rc"
colcon build --packages-select arena_autonomy --event-handlers console_direct+ > "$base/final_build.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/final_build.exit"
[ "$rc" = 0 ] || exit "$rc"
env -u PYTHONPATH bash -c 'source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; /usr/bin/python3 -B artifacts/validation/2026-10-29/rear_lane_preference_connection_v1/check_installed.py' > "$base/final_installed.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/final_installed.exit"
[ "$rc" = 0 ] || exit "$rc"
bash artifacts/validation/2026-10-29/rear_lane_preference_gazebo_v1/preflight.sh
