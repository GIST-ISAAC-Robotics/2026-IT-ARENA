#!/usr/bin/env bash
# 출력 비활성 연결부의 공유 검사와 관련 회귀. 실제 장치 I/O 없음.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/actuation_bench_v1
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
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_smoke.py tests/test_actuation_ingress.py tests/test_actuation_tx_policy.py tests/test_actuation_host_observer.py tests/test_actuation_source_time_review.py tests/test_actuation_separate_clock.py > "$evidence/tests.log" 2>&1
