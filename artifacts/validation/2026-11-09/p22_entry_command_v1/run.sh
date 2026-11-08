#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/p22_entry_command_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B "$A/probe.py" >"$A/run_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" >"$A/run_v1.exit"
exit "$rc"
