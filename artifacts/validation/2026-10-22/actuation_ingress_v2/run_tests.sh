#!/usr/bin/env bash
set -euo pipefail
task_root=$(cd -- "$(dirname -- "$0")/../../../.." && pwd)
cd -- "$task_root"
evidence=artifacts/validation/2026-10-22/actuation_ingress_v2
test ! -e "$evidence/tests_exit.txt"
task_sources=(src/arena_vehicle_interface/portable/*.hpp scripts/actuation_ingress_driver.cpp \
  scripts/actuation_smoke_host.cpp src/arena_vehicle_interface/firmware_smoke/main/* \
  tests/test_actuation_ingress.py tests/test_actuation_smoke.py)
sha256sum "${task_sources[@]}" > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot.tar.gz" "${task_sources[@]}"
finish() {
  result=$?
  sha256sum "${task_sources[@]}" > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/tests_exit.txt"
}
trap finish EXIT
python3 -m pytest -q tests/test_actuation_ingress.py tests/test_actuation_smoke.py
