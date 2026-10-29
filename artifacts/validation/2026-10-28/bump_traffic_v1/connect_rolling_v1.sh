#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/bump_traffic_v1
run_checked() {
    local name="$1"
    shift
    test ! -e "$base/${name}.log"
    set +e
    "$@" > "$base/${name}.log" 2>&1
    local result=$?
    set -e
    printf '%s\n' "$result" > "$base/${name}.exit"
    if [ "$result" -ne 0 ]; then tail -n 60 "$base/${name}.log"; exit "$result"; fi
    tail -n 6 "$base/${name}.log"
}
run_checked connected_contracts_v1 /usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_rolling_clearance_state.py tests/test_rolling_clearance_provenance.py tests/test_positive_clearance_state.py tests/test_anchor_clearance_probe.py tests/test_recovery_threat_capture.py tests/test_traffic_recovery_contract.py tests/test_surface_reobservation_connection.py tests/test_small_alignment_provenance.py tests/test_rotation_reobservation_provenance.py tests/test_source_contract_helpers.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_source_provenance_r0_plus_b3.py tests/test_same_path_speed_recheck.py tests/test_g1_guard_partial_coupling.py tests/test_traffic_release_fringe.py
run_checked connected_build_v1 colcon build --symlink-install --packages-select arena_autonomy
source install/local_setup.bash
run_checked connected_failure_v1 /usr/bin/python3 -B artifacts/validation/2026-10-28/rotation_reobservation_review_v1/validate_connected.py "$base/run2/follow_decisions.jsonl" "$base/rolling_failure_v1/decisions.jsonl" --output "$base/connected_failure_v1"
run_checked connected_run4_v1 /usr/bin/python3 -B artifacts/validation/2026-10-28/rotation_reobservation_review_v1/validate_connected.py artifacts/validation/2026-10-28/rotation_stress_v1/run4_course/follow_decisions.jsonl "$base/rolling_run4_v1/decisions.jsonl" --output "$base/connected_run4_v1"
