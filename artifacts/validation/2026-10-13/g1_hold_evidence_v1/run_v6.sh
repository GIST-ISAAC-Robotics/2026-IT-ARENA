#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_hold_evidence_v1
test ! -e "$base/v6_tests.log"
python3 -B -m pytest -q tests/test_g1_case_evidence.py tests/test_g1_hold_evidence.py tests/test_g1_dense_stop.py tests/test_g1_stop_pose_coverage.py tests/test_g1_trace_timing.py tests/test_g1_traffic_coverage_audit.py > "$base/v6_tests.log" 2>&1
tail -3 "$base/v6_tests.log"
for item in 'gapped_boundary_v3/side_left_s0 side_left_s0 g1_side_occlusion_v1' 'g1_right_candidate_v1/side_right_s0 side_right_s0 g1_side_occlusion_v1' 'g1_position_matrix_v1/side_left_s1 side_left_s1 g1_side_occlusion_v1' 'g1_candidate_regression_v1/initial_s0 initial_old_s0 g1_traffic_extensions_v1' 'g1_curve_candidate_v1/first_curve_s0 curve_s0 g1_traffic_extensions_v1' 'g1_dense_stop_v1/initial_s0 initial_new_s0 g1_traffic_extensions_v1'; do
  read -r run name config <<< "$item"
  python3 -B scripts/audit_g1_case_evidence.py "artifacts/validation/2026-10-13/$run" --config "config/tests/$config.json" --output "$base/$name.v6.json"
done
