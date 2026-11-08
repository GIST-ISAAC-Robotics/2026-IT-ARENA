#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/rear_overtake_continuity_review_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
for step in tests base candidate; do
  if [[ "$step" == tests ]]; then
    bash "$ENV" python3 -B -m pytest -q "$A/test_candidate.py" > "$A/tests.log" 2>&1
  else
    bash "$ENV" python3 -B "$A/replay.py" "$step" > "$A/replay_$step.log" 2>&1
  fi
  code=$?
  printf '%s\n' "$code" > "$A/$step.exit.txt"
  [[ "$code" == 0 ]] || exit "$code"
done
