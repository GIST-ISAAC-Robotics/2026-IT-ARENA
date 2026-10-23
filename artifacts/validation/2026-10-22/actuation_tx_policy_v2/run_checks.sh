#!/usr/bin/env bash
# 송신 정책 후보 v2(v1 변이체 1개 미검출 뒤 STOP ack 순서 시험 보강): 결정론 선로/실제 호스트 콜백 감사 + 관련·변이체·전체 회귀. 보드/UART/ROS 노드 실행 없음.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
unset ARENA_TX_PORTABLE_OVERRIDE
evidence=artifacts/validation/2026-10-22/actuation_tx_policy_v2
test ! -e "$evidence/checks_exit.txt"
task_sources=(src/arena_vehicle_interface/portable/*.hpp src/arena_vehicle_interface/firmware_smoke/main/* \
  scripts/actuation_tx_driver.cpp scripts/audit_actuation_tx_policy.py scripts/check_actuation_tx_mutants.py \
  scripts/actuation_smoke_host.cpp scripts/audit_actuation_callback_budget.py \
  tests/test_actuation_tx_policy.py tests/test_actuation_smoke.py \
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
python3 -B scripts/audit_actuation_tx_policy.py --output "$evidence/audit" > "$evidence/audit.log" 2>&1
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_tx_policy.py tests/test_actuation_smoke.py \
  tests/test_actuation_ingress.py tests/test_actuation_callback_budget.py tests/test_actuation_link_budget.py \
  > "$evidence/related_tests.log" 2>&1
python3 -B scripts/check_actuation_tx_mutants.py --output "$evidence/mutants" > "$evidence/mutants.log" 2>&1
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/full_tests.log" 2>&1
