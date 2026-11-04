#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$A/finish.py" > "$A/finish.log" 2>&1
echo "CONTRACT=$?" > "$A/final.exit"
python3 -B -m pytest -q -p no:cacheprovider --tb=line tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_source_provenance_r0_plus_b3.py \
  tests/test_observed_transition.py tests/test_observed_motion_gates.py tests/test_traffic_merge.py > "$A/tests_final.log" 2>&1
echo "TESTS=$?" >> "$A/final.exit"
