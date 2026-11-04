#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_forward_pace_v3
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_observed_forward_pace.py tests/test_low_motion_reserve.py tests/test_follow_closing_reserve.py tests/test_observed_rejoin.py tests/test_bypass_handoff.py tests/test_observed_motion_gates.py tests/test_g1_handoff_eval.py tests/test_source_provenance_r0_plus_b3.py tests/test_continuous_response_eval.py > "$A/tests_focus.log" 2>&1
python3 -B "$A/replay.py" "$A/a3_replay.json" > "$A/replay.log" 2>&1
