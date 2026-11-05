#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-05/moving_pass_v1
N="${1:-unit2}"
if [[ -e "$A/$N.log" || -e "$A/$N.exit" ]]; then echo 'choose a new output name'; exit 2; fi
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh /usr/bin/python3 -B -m pytest -q -p no:cacheprovider \
 tests/test_moving_pass.py tests/test_bypass_entry_guide.py tests/test_bypass_entry_trial.py \
 tests/test_bypass_handoff.py tests/test_g1_handoff_eval.py tests/test_continuous_forward_pace.py \
 tests/test_bypass_yield_progress.py tests/test_g1_traffic_extensions.py > "$A/$N.log" 2>&1
code=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$code" > "$A/$N.exit"
tail -n 28 "$A/$N.log"
exit "$code"
