#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_gap_envelope_review_v2
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B -m pytest "$A/test_candidate.py" -q > "$A/counterexamples.log" 2>&1
r=$?
printf '%s\n' "$r" > "$A/counterexamples.exit"
exit "$r"
