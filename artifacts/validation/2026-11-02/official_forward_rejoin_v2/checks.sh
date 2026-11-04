#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_forward_rejoin_v2
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_observed_rejoin.py tests/test_bypass_handoff.py tests/test_observed_motion_gates.py tests/test_g1_handoff_eval.py tests/test_source_provenance_r0_plus_b3.py > "$A/tests_focus2.log" 2>&1
