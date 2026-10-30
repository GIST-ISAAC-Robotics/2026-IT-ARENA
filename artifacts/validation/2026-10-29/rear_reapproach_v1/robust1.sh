#!/usr/bin/env bash
# 합성 센서 복합 오차(range ±3 cm, LiDAR yaw -0.5°, gyro +1°/s·σ0.02, 모터축 양자화)와 빠른 앞차 사후 진단.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-29/rear_reapproach_v1
cases="A_recede_left B_reapproach_left C_lead_stop_left D_opposite_residual_left"
bash $base/run_py.sh $base/probe_rear.py --trace --sensor-profile combined --cases $cases --output $base/robust1_off > $base/robust1_off.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --sensor-profile combined --rear-lane-keep --cases $cases --output $base/robust1_on > $base/robust1_on.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --cases G_fast_lead_left --output $base/fast1_off > $base/fast1_off.log 2>&1 &
bash $base/run_py.sh $base/probe_rear.py --trace --rear-lane-keep --cases G_fast_lead_left --output $base/fast1_on > $base/fast1_on.log 2>&1 &
wait
