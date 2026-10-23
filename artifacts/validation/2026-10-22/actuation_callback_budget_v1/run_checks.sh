#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd -- "$(dirname -- "$0")/../../../.." && pwd)
cd -- "$task_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-22/actuation_callback_budget_v1
test ! -e "$evidence/checks_exit.txt"
task_sources=(scripts/audit_actuation_callback_budget.py tests/test_actuation_callback_budget.py \
  scripts/audit_actuation_link_budget.py scripts/actuation_conformance.py \
  src/arena_vehicle_interface/arena_vehicle_interface/actuation*.py)
sha256sum "${task_sources[@]}" > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot.tar.gz" "${task_sources[@]}"
finish() {
  result=$?
  sha256sum "${task_sources[@]}" > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
}
trap finish EXIT
python3 -B scripts/audit_actuation_callback_budget.py --output "$evidence/audit"
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_callback_budget.py tests/test_actuation_link_budget.py
