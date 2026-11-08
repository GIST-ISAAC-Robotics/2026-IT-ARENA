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
run_step build_v1 colcon build --symlink-install --packages-select arena_autonomy arena_bringup || exit "$?"
run_step geometry_v1 python3 -B "$A/geometry.py" || exit "$?"
run_step register python3 -B "$A/register.py" || exit "$?"
bash "$A/run.sh" persistent_block
