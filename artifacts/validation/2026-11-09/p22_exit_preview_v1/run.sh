#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/p22_exit_preview_v1
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/probe.py" >"$A/probe_v2.log" 2>&1
rc=$?
printf '%s\n' "$rc" >"$A/probe_v2.exit"
exit "$rc"
