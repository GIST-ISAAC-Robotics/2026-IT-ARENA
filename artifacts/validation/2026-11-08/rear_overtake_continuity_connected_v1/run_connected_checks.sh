#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/rear_overtake_continuity_connected_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B -m pytest tests/test_parallel_measured_continuity.py tests/test_parallel_lane_trial.py tests/test_parallel_side_departure.py tests/test_pass_release_trial.py tests/test_fresh_parallel_trial.py tests/test_m3_parallel_straight.py tests/test_signed_forward_drive.py tests/test_bypass_signed_hold_trial.py tests/test_source_provenance_r0_plus_b3.py tests/test_traffic_road_route.py tests/test_traffic_actor_integration.py -q > "$A/regression.log" 2>&1
status=$?
printf '%s\n' "$status" > "$A/regression.exit.txt"
[ "$status" -eq 0 ] || exit "$status"
bash "$ENV" python3 -B "$A/replay_connected.py" > "$A/replay_connected.log" 2>&1
status=$?
printf '%s\n' "$status" > "$A/replay_connected.exit.txt"
exit "$status"
