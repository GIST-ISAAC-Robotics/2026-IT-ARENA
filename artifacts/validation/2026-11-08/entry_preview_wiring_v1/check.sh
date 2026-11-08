#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/entry_preview_wiring_v1
python3 -B -I "$A/check_wiring.py" > "$A/check.log" 2>&1
status=$?; printf '%s\n' "$status" > "$A/check.exit"; exit "$status"
