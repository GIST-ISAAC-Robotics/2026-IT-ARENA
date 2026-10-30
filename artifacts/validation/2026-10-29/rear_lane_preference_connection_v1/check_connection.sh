#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_lane_preference_connection_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/tests.log"
set +e
/usr/bin/python3 -B -m pytest -q -rs -p no:cacheprovider tests/test_observed_lane_preference_trial.py tests/test_observed_lane_preference_provenance.py tests/test_rear_lane_keep_trial.py tests/test_rear_lane_keep_provenance.py \
  tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_traffic_path_recovery.py tests/test_traffic_release_fringe.py \
  tests/test_same_path_speed_recheck.py tests/test_g1_guard_partial_coupling.py tests/test_traffic_recovery_contract.py \
  tests/test_actor_tail_provenance.py tests/test_g2_handoff_provenance.py tests/test_source_provenance_r0_plus_b3.py \
  tests/test_source_contract_helpers.py tests/test_anchor_set_provenance.py tests/test_corner_follow.py tests/test_follow_profile.py \
  tests/test_follow_profile_bundle.py tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py \
  tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_jetson_bundle_dependencies.py \
  tests/test_bypass_handoff.py tests/test_g1_partial_traffic.py > "$base/tests.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/tests.exit"
[ "$rc" = 0 ] || exit "$rc"
for kind in off on; do
  /usr/bin/python3 -B scripts/replay_follow_trace.py "artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/drive_$kind/follow_decisions.jsonl" --output "$base/replay_$kind.json" > "$base/replay_$kind.log" 2>&1
  rc=$?
  printf '%s\n' "$rc" > "$base/replay_$kind.exit"
  [ "$rc" = 0 ] || exit "$rc"
done
colcon build --packages-select arena_autonomy --event-handlers console_direct+ > "$base/build.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/build.exit"
[ "$rc" = 0 ] || exit "$rc"
env -u PYTHONPATH bash -c 'source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; /usr/bin/python3 -B artifacts/validation/2026-10-29/rear_lane_preference_connection_v1/check_installed.py' > "$base/installed.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/installed.exit"
exit "$rc"
