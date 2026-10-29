#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider tests/test_same_path_side_scope.py tests/test_anchor_clearance_probe.py tests/test_positive_clearance_state.py tests/test_surface_anchor_probe.py tests/test_recovery_threat_capture.py tests/test_partial_surface_alignment.py tests/test_same_path_speed_recheck.py tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_same_path_gap_probe.py tests/test_traffic_release_fringe.py tests/test_traffic_path_recovery.py tests/test_source_provenance_r0_plus_b3.py tests/test_anchor_set_provenance.py tests/test_traffic_recovery_contract.py > artifacts/validation/2026-10-28/traffic_side_scope_v1/static_v2.log 2>&1
