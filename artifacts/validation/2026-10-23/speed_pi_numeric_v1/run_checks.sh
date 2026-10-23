#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
export PYTHONPATH="$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/speed_pi_numeric_v1
test ! -e "$evidence/checks_exit.txt"
sources=(src/arena_vehicle_interface/arena_vehicle_interface/mcu_speed_loop.py tests/test_speed_pi_numeric.py)
sha256sum "${sources[@]}" > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot.tar.gz" "${sources[@]}"
finish() {
  result=$?
  sha256sum "${sources[@]}" > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
}
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_speed_pi_numeric.py > "$evidence/tests.log" 2>&1
