#!/usr/bin/env bash
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
ART=artifacts/validation/2026-11-08/rear_face_motion_shadow_v2
CASE=${1:?case}
python3 -B "$ART/probe.py" "$CASE" > "$ART/${CASE}_v2.log" 2>&1
RESULT=$?
printf '%s\n' "$RESULT" > "$ART/${CASE}_v2.exit.txt"
exit "$RESULT"
