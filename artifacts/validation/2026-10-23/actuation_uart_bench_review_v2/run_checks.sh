#!/usr/bin/env bash
# 최종 검토 결함 수정: 이전 판본 결함 재현 → 수정본 관련 회귀 → 변이 검사. 실제 UART/보드 실행 없음.
set -uo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/actuation_uart_bench_review_v2
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
flags=(-std=c++17 -Wall -Wextra -Werror -pedantic -fno-exceptions -fno-rtti -ffp-contract=off -O1 -fsanitize=address,undefined -fno-omit-frame-pointer)
cases=(-Isrc/arena_vehicle_interface/firmware_smoke/main -Isrc/arena_vehicle_interface/firmware_bench/main)
# 1. 결함 판본: 첫 전체 회귀(actuation_uart_bench_v1)의 원시 스냅샷에서 이식 헤더만 꺼낸다.
mkdir -p "$evidence/defect_source"
tar -xzf artifacts/validation/2026-10-23/actuation_uart_bench_v1/source_snapshot.tar.gz -C "$evidence/defect_source" src/arena_vehicle_interface/portable
sha256sum "$evidence/defect_source/src/arena_vehicle_interface/portable/actuation_uart_bench.hpp" src/arena_vehicle_interface/portable/actuation_uart_bench.hpp > "$evidence/adapter_headers.sha256"
g++ "${flags[@]}" -I"$evidence/defect_source/src/arena_vehicle_interface/portable" "${cases[@]}" scripts/uart_bench_defect_repro.cpp -o "$evidence/repro_defect" > "$evidence/repro_defect_compile.log" 2>&1
"$evidence/repro_defect" > "$evidence/repro_defect.log" 2>&1
defect_exit=$?
g++ "${flags[@]}" -Isrc/arena_vehicle_interface/portable "${cases[@]}" scripts/uart_bench_defect_repro.cpp -o "$evidence/repro_fixed" > "$evidence/repro_fixed_compile.log" 2>&1
"$evidence/repro_fixed" > "$evidence/repro_fixed.log" 2>&1
fixed_exit=$?
printf 'defect_exit=%s\nfixed_exit=%s\n' "$defect_exit" "$fixed_exit" > "$evidence/repro_exit.txt"
# 2. 관련 회귀(공유 헤더를 쓰는 smoke 포함). 3. 변이 검사.
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_uart_bench.py tests/test_actuation_smoke.py > "$evidence/related_tests.log" 2>&1
tests_exit=$?
python3 -B scripts/check_uart_bench_mutants.py --output "$evidence/mutants" > "$evidence/mutants.log" 2>&1
mutants_exit=$?
printf 'tests_exit=%s\nmutants_exit=%s\n' "$tests_exit" "$mutants_exit" >> "$evidence/repro_exit.txt"
test "$defect_exit" -eq 1 && test "$fixed_exit" -eq 0 && test "$tests_exit" -eq 0 && test "$mutants_exit" -eq 0
