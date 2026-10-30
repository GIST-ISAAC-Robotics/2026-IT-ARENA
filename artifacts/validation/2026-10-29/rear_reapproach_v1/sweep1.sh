#!/usr/bin/env bash
# 진단 sweep: 현재 코드에서 옆차 횡위치별로 옆→뒤 전이가 생기는지(A 왼쪽 일정) 확인한다.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-29/rear_reapproach_v1
for y in 0.13 0.15 0.17 0.20 0.25 0.32; do
  bash $base/run_py.sh $base/probe_rear.py --cases A_recede_left B_reapproach_left --side-y $y --output $base/sweep1/y$y
done
