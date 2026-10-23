#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-21/actuation_link_budget_v1
if [[ -e "$evidence/tests.log" || -e "$evidence/export" ]]; then
    printf 'Fresh evidence path required\n' >&2
    exit 2
fi
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot_before.tar.gz" src scripts tests config
finish() {
    result=$?
    find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
    diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
    printf 'EXIT_STATUS=%s\n' "$result" > "$evidence/runner_status.txt"
}
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/tests.log" 2>&1
python3 -B scripts/audit_actuation_link_budget.py --output "$evidence/export" > "$evidence/export.log" 2>&1
