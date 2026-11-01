#!/bin/bash
# 저장 입력 재생만 수행한다. Gazebo/빌드/소스 변경 없음. 진행 중 시험을 방해하지 않도록 최저 우선순위.
set -eu
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/qualifying_corner_diagnosis_v1"
SNAP="$HOME/arena_scratch/qc_diag/snap_combined"
mkdir -p "$D/replay"
for e in baseline truth_history truth_speed truth_both; do
  nice -n 19 python3 -I "$D/replay_diag.py" combined_failed "$SNAP" "$e" 44.9 "$D/replay/combined_$e.json"
done
