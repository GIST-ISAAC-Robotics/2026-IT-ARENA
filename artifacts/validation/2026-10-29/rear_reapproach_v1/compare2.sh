#!/usr/bin/env bash
# 수정 후보(any-increase 기준) 재비교: 명목 전체, 복합 오차 4장면, 빠른 앞차. OFF 결과는 current3/robust1_off/fast1_off.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-29/rear_reapproach_v1
cases="A_recede_left B_reapproach_left C_lead_stop_left D_opposite_residual_left"
bash $base/run_py.sh $base/probe_rear.py --trace --rear-lane-keep --output $base/lane4 > $base/lane4.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --sensor-profile combined --rear-lane-keep --cases $cases B_reapproach_right --output $base/robust2_on > $base/robust2_on.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --sensor-profile combined --cases B_reapproach_right --output $base/robust2_off > $base/robust2_off.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --rear-lane-keep --cases G_fast_lead_left --output $base/fast2_on > $base/fast2_on.log 2>&1 &
bash $base/unit_tests.sh unit3 tests/test_rear_lane_keep_trial.py &
wait
