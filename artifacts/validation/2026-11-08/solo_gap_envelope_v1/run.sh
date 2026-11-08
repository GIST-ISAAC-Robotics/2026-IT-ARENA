#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_gap_envelope_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B "$A/replay.py" > "$A/replay.log" 2>&1
r=$?;printf '%s\n' "$r" > "$A/replay.exit";exit "$r"
