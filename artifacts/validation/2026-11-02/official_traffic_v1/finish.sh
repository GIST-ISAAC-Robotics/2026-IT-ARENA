#!/usr/bin/env bash
# 평가 재계산, 소스 계약, 전체 회귀. 시뮬레이터 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
for n in p1 r1 c1; do python3 -B "$A/goal_eval.py" "$A/$n" src/arena_gazebo/worlds/it_arena_official; done > "$A/goal_recompute.log" 2>&1
echo "GOAL=$?" > "$A/finish.exit"
python3 -B "$A/finish.py" > "$A/finish.log" 2>&1
echo "FINISH=$?" >> "$A/finish.exit"
python3 -B -m pytest -q -p no:cacheprovider tests --tb=line > "$A/tests_full.log" 2>&1
echo "FULL=$?" >> "$A/finish.exit"
