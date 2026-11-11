#!/usr/bin/env bash
set -u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/confirmed_passage_fringe_r61_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$A/tests_v2.log" || -e "$A/replay" ]]; then exit 4; fi
bash "$ENV" /usr/bin/python3 -B -m pytest -q tests/test_confirmed_passage_fringe.py tests/test_traffic_release_fringe.py tests/test_released_rear_sweep.py tests/test_released_fringe_budget.py tests/test_rear_fringe_trial.py tests/test_guard_curve_trial.py tests/test_pass_release_trial.py tests/test_source_provenance_r0_plus_b3.py > "$A/tests_v2.log" 2>&1
tests=$?
printf 'TESTS_EXIT=%s\n' "$tests" > "$A/checks_v2.exit"
if [[ $tests -ne 0 ]]; then exit "$tests"; fi
bash "$ENV" /usr/bin/python3 -B "$A/verify.py" > "$A/replay.log" 2>&1
replay=$?
printf 'REPLAY_EXIT=%s\n' "$replay" >> "$A/checks_v2.exit"
exit "$replay"
