#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/rear_overtake_continuity_connected_v1
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/replay_cost_before.py" > "$A/cost_before.log" 2>&1
r=$?;printf '%s\n' "$r" > "$A/cost_before.exit";exit "$r"
