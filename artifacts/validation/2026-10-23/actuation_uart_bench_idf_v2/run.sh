#!/usr/bin/env bash
# v1 기호 검사 기대값 수정 뒤: 관련 정적/공유 사례 회귀 → 대상 빌드 1회. flash/monitor 없음.
set -uo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
evidence=artifacts/validation/2026-10-23/actuation_uart_bench_idf_v2
export PYTHONDONTWRITEBYTECODE=1
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_uart_bench.py > "$evidence/related_tests.log" 2>&1
printf '%s\n' "$?" > "$evidence/related_tests_exit.txt"
bash scripts/build_actuation_bench.sh "$evidence" /home/jinhyeong/.cache/it-arena/idf-6.1-smoke/bench-source-v2 > "$evidence/build.log" 2>&1
printf 'wrapper_build_exit=%s\n' "$?" >> "$evidence/related_tests_exit.txt"
