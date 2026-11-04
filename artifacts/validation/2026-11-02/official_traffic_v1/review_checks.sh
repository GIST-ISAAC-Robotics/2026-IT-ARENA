#!/usr/bin/env bash
# 최종 검토 뒤 기존 자료 재계산과 변경 영향 시험. 시뮬레이터 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
for n in p1 r1 c1; do python3 -B "$A/goal_eval.py" "$A/$n" src/arena_gazebo/worlds/it_arena_official || echo "GOAL_FAILED=$n"; done > "$A/goal_recompute2.log" 2>&1
rm -f "$A/goal_eval_negative.json"
python3 -B "$A/negative.py" > "$A/negative2.log" 2>&1
echo "NEGATIVE=$?" > "$A/review.exit"
python3 -B -m pytest -q -p no:cacheprovider --tb=line tests/test_traffic_merge.py tests/test_observed_transition.py tests/test_source_provenance_r0_plus_b3.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py > "$A/tests_review.log" 2>&1
echo "TESTS=$?" >> "$A/review.exit"
