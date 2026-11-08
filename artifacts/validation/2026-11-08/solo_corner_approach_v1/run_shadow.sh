#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_corner_approach_v1
for k in contact r14 straight; do
    bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/shadow.py" "$k" > "$A/$k.log" 2>&1
    status=$?;printf '%s\n' "$status" > "$A/$k.exit"; [ "$status" -eq 0 ] || exit "$status"
done
