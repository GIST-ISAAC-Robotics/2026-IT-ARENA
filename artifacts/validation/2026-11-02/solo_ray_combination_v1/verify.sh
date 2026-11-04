#!/usr/bin/env bash
set -eu
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/solo_ray_combination_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
step() { name=$1; shift; set +e; "$@" > "$A/$name.log" 2>&1; code=$?; set -e; printf 'ACTUAL_EXIT_CODE=%s\n' "$code" > "$A/$name.exit"; [ "$code" -eq 0 ]; }
step tests bash "$ENV" python3 -B -m pytest -q tests/test_ray_wall_continuity.py tests/test_boundary_bridge.py tests/test_turn_speed_preview.py tests/test_turn_preview_gate.py tests/test_solo_alternative_speed.py tests/test_source_provenance_r0_plus_b3.py
step build bash "$ENV" colcon build --packages-select arena_bringup --symlink-install --allow-overriding arena_bringup
step prepare bash "$ENV" python3 -B "$A/run.py" --prepare-only
