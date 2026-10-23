#!/usr/bin/env bash
# 별도 시계 offer의 오프라인 후보. UART/ROS/Gazebo/보드 실행 없음.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/actuation_offer_clock_v1
test ! -e "$evidence/checks_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar --exclude='__pycache__' -czf "$evidence/source_snapshot.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
}
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_offer_clock.py tests/test_clock_bounds.py tests/test_clock_exchange_audit.py tests/test_clock_freshness.py tests/test_clock_pair_audit.py tests/test_actuation_contract.py > "$evidence/tests.log" 2>&1
python3 -B scripts/audit_actuation_offer_clock.py --output "$evidence/audit" > "$evidence/audit_stdout.log" 2>&1
