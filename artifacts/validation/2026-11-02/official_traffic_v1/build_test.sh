#!/usr/bin/env bash
# 장면 등록·빌드·영향권 시험. 시뮬레이터 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
NAME=${1:-tests_focus}
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$A/build.log" 2>&1
echo "BUILD=$?" > "$A/$NAME.exit"
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$A/prepare.py" > "$A/prepare.log" 2>&1
echo "PREPARE=$?" >> "$A/$NAME.exit"
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_observed_motion_gates.py tests/test_traffic_merge.py tests/test_g1_traffic_eval.py \
  tests/test_race_traffic_course.py tests/test_rolling_lead_course.py tests/test_official_signal_control.py tests/test_observed_transition.py > "$A/$NAME.log" 2>&1
echo "TESTS=$?" >> "$A/$NAME.exit"
