#!/usr/bin/env bash
# 앞 모서리 기준·통로 차단 물체 제외 뒤 재비교(compare3). compare2 결과는 보존.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-29/rear_reapproach_v1
cases="A_recede_left B_reapproach_left C_lead_stop_left D_opposite_residual_left"
bash $base/run_py.sh $base/probe_rear.py --trace --rear-lane-keep --output $base/lane5 > $base/lane5.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --sensor-profile combined --rear-lane-keep --cases $cases B_reapproach_right --output $base/robust3_on > $base/robust3_on.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --sensor-profile combined --cases A_recede_right --output $base/robust3_off > $base/robust3_off.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --rear-lane-keep --cases G_fast_lead_left --output $base/fast3_on > $base/fast3_on.log 2>&1 &
bash $base/unit_tests.sh unit4 tests/test_rear_lane_keep_trial.py &
wait
