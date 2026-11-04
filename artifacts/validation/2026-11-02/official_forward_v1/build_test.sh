#!/usr/bin/env bash
# 소스 수정 뒤 빌드와 영향권 시험. 시뮬레이터 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_forward_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$A/build.log" 2>&1
echo "BUILD=$?" > "$A/build.exit"
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_observed_motion_gates.py tests/test_traffic_merge.py tests/test_g1_traffic_eval.py tests/test_race_traffic_course.py tests/test_rolling_lead_course.py tests/test_official_signal_control.py > "$A/tests_gates.log" 2>&1
echo "TESTS=$?" >> "$A/build.exit"
