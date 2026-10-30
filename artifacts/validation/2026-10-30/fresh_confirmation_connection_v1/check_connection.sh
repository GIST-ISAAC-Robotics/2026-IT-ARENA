#!/usr/bin/env bash
# 단계 2: 관련 회귀 → 과거 OFF/timed/observed 기록 동일 재생 → 런타임/오프라인 v4 대조 → 빌드 → 설치 대조.
# 어느 단계든 실패하면 다음 단계로 넘어가지 않는다.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/fresh_confirmation_connection_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
tag=${1:?run tag required}
out="$base/connection_$tag"
test ! -e "$out"
mkdir "$out"
step() {
  local name=$1; shift
  set +e
  "$@" > "$out/$name.log" 2>&1
  local rc=$?
  set -e
  printf '%s\n' "$rc" > "$out/$name.exit"
  [ "$rc" = 0 ] || exit "$rc"
}
step snapshot /usr/bin/python3 -B "$base/snapshot_sources.py" "$out/sources.zip"
step tests /usr/bin/python3 -B -m pytest -q -rs -p no:cacheprovider \
  tests/test_fresh_parallel_trial.py tests/test_fresh_confirmation_provenance.py \
  tests/test_measured_lane_preference_trial.py tests/test_measured_lane_preference_provenance.py \
  tests/test_observed_lane_preference_trial.py tests/test_observed_lane_preference_provenance.py \
  tests/test_rear_lane_keep_trial.py tests/test_rear_lane_keep_provenance.py \
  tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_traffic_path_recovery.py tests/test_traffic_release_fringe.py \
  tests/test_same_path_speed_recheck.py tests/test_g1_guard_partial_coupling.py tests/test_traffic_recovery_contract.py \
  tests/test_actor_tail_provenance.py tests/test_g2_handoff_provenance.py tests/test_source_provenance_r0_plus_b3.py \
  tests/test_source_contract_helpers.py tests/test_anchor_set_provenance.py tests/test_corner_follow.py tests/test_follow_profile.py \
  tests/test_follow_profile_bundle.py tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py \
  tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_jetson_bundle_dependencies.py \
  tests/test_bypass_handoff.py tests/test_g1_partial_traffic.py
step replay_off /usr/bin/python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/drive_off/follow_decisions.jsonl --output "$out/replay_off.json"
step replay_timed /usr/bin/python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/drive_on/follow_decisions.jsonl --output "$out/replay_timed.json"
step replay_observed /usr/bin/python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-29/rear_lane_preference_gazebo_v1/drive_on/follow_decisions.jsonl --output "$out/replay_observed.json"
step replay_measured /usr/bin/python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1/drive_on/follow_decisions.jsonl --output "$out/replay_measured.json"
step runtime_vs_body /usr/bin/python3 -B "$base/compare.py" "$out/runtime_vs_body.json"
step build colcon build --packages-select arena_autonomy --event-handlers console_direct+
step installed env -u PYTHONPATH bash -c 'source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; /usr/bin/python3 -B artifacts/validation/2026-10-30/fresh_confirmation_connection_v1/check_installed.py'
