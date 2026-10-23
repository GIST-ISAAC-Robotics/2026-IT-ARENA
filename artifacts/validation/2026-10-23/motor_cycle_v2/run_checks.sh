#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/motor_cycle_v2
test ! -e "$evidence/checks_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar --exclude=__pycache__ -czf "$evidence/source_snapshot.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
}
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_motor_cycle_cpp.py tests/test_speed_pi_cpp.py tests/test_speed_pi_numeric.py tests/test_actuation_contract.py tests/test_actuation_smoke.py > "$evidence/tests.log" 2>&1
python3 -B scripts/check_motor_cycle_mutants.py --output "$evidence/mutants" > "$evidence/mutants.log" 2>&1
