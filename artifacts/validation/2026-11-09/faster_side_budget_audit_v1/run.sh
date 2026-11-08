#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/faster_side_budget_audit_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B "$A/analyze.py" > "$A/analyze_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/analyze_v1.exit"
exit "$rc"
