#!/usr/bin/env bash
# 후보 연결 뒤 관련 회귀(출처 사슬·보존 목록·선택기)와 arena_autonomy 빌드·설치 모듈 확인.
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_reapproach_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/checks1_tests.log" || exit 73
/usr/bin/python3 -B -m pytest -q -rs -p no:cacheprovider tests/test_rear_lane_keep_trial.py tests/test_rear_lane_keep_provenance.py \
  tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_traffic_path_recovery.py tests/test_traffic_release_fringe.py \
  tests/test_same_path_speed_recheck.py tests/test_g1_guard_partial_coupling.py tests/test_traffic_recovery_contract.py \
  tests/test_actor_tail_provenance.py tests/test_g2_handoff_provenance.py tests/test_source_provenance_r0_plus_b3.py \
  tests/test_source_contract_helpers.py tests/test_anchor_set_provenance.py tests/test_corner_follow.py tests/test_follow_profile.py \
  tests/test_follow_profile_bundle.py tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py \
  tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_jetson_bundle_dependencies.py \
  tests/test_bypass_handoff.py tests/test_g1_partial_traffic.py > "$base/checks1_tests.log" 2>&1
echo $? > "$base/checks1_tests.exit"
colcon build --packages-select arena_autonomy --event-handlers console_direct+ > "$base/checks1_build.log" 2>&1
echo $? > "$base/checks1_build.exit"
source install/local_setup.bash
env -u PYTHONPATH bash -c 'source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; /usr/bin/python3 -B -c "
import arena_autonomy.local_follow as lf, arena_autonomy.rear_lane_keep_trial as rl, arena_autonomy.local_pursuit as lp, hashlib, pathlib, json
src=pathlib.Path(\"src/arena_autonomy/arena_autonomy\")
same={n: hashlib.sha256(pathlib.Path(m.__file__).read_bytes()).hexdigest()==hashlib.sha256((src/(n+\".py\")).read_bytes()).hexdigest() for n,m in ((\"local_follow\",lf),(\"rear_lane_keep_trial\",rl),(\"local_pursuit\",lp))}
print(json.dumps(dict(installed_matches_source=same, default_off=lf.FollowConfig().rear_lane_keep_trial is False, files={n:m.__file__ for n,m in ((\"local_follow\",lf),(\"rear_lane_keep_trial\",rl))})))
"' > "$base/checks1_installed.log" 2>&1
echo $? > "$base/checks1_installed.exit"
