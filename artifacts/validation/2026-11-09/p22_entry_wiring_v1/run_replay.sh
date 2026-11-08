#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/p22_entry_wiring_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B "$A/replay.py" >"$A/replay_v2.log" 2>&1
rc=$?
printf '%s\n' "$rc" >"$A/replay_v2.exit"
exit "$rc"
