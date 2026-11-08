#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/rear_face_motion_connected_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
run_step() {
    name=$1; shift
    bash "$ENV" "$@" > "$A/$name.log" 2>&1
    status=$?
    printf '%s\n' "$status" > "$A/$name.exit.txt"
    if [ "$status" -ne 0 ]; then return "$status"; fi
}
run_step regression_v2 python3 -m pytest tests/test_bypass_corridor_trial.py::test_default_launches_and_files_do_not_select_the_policy -q || exit "$?"
run_step stationary_replay_v1 python3 -B "$A/replay.py" stationary || exit "$?"
run_step restart_replay_v1 python3 -B "$A/replay.py" restart || exit "$?"
run_step moving_replay_v1 python3 -B "$A/replay_moving.py" || exit "$?"
