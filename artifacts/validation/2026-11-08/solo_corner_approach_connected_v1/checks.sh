#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_corner_approach_connected_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
step() { name=$1;shift; bash "$ENV" "$@" > "$A/$name.log" 2>&1; s=$?;printf '%s\n' "$s" > "$A/$name.exit";return "$s"; }
step regression python3 -m pytest -q tests/test_solo_corner_approach.py tests/test_solo_straight_budget.py tests/test_source_provenance_r0_plus_b3.py || exit "$?"
step replay python3 -B "$A/replay.py" || exit "$?"
step build colcon build --symlink-install --packages-select arena_autonomy arena_bringup || exit "$?"
