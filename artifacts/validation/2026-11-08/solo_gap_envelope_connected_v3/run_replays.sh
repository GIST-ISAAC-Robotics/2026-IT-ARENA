#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3
for mode in base connected; do
  bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/replay.py" "$mode" > "$A/replay_$mode.log" 2>&1
  r=$?; printf '%s\n' "$r" > "$A/replay_$mode.exit"
  [ "$r" -eq 0 ] || exit "$r"
done
