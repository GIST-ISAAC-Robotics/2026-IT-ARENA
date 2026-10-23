#!/usr/bin/env bash
set -eo pipefail
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/race_actuation_v2
test ! -e "$evidence/exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar --exclude=__pycache__ -czf "$evidence/source_snapshot.tar.gz" src scripts tests config
finish() {
  result=$?; trap - EXIT
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_diff.txt" || result=1
  printf '%s\n' "$result" > "$evidence/exit.txt"
  exit "$result"
}
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_race_actuation_contract.py \
 tests/test_race_permission.py tests/test_official_start_signal.py tests/test_vision_process.py \
 tests/test_actuation_contract.py --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
