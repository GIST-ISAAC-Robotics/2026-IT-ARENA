#!/bin/bash
# 저장 입력 재생만 수행한다. 성공 실행에서 운동 입력 민감도와 정지 장면 후보를 본다.
set -eu
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/qualifying_corner_diagnosis_v1"
SNAP="$HOME/arena_scratch/qc_diag/snap_combined"
nice -n 19 python3 -I "$D/stop_scene.py" "$SNAP" "$D/stop_scene.json" > "$D/stop_scene.log" 2>&1 || echo STOP_SCENE_FAIL
for run in rise_only turn_only; do
  for e in baseline truth_history; do
    nice -n 19 python3 -I "$D/replay_diag.py" "$run" "$SNAP" "$e" 8.0 "$D/replay/${run}_$e.json"
  done
done
nice -n 19 python3 -I "$D/replay_diag.py" combined_failed "$SNAP" truth_history 8.0 "$D/replay/combinedfull_truth_history.json"
nice -n 19 python3 -I "$D/replay_diag.py" combined_failed "$SNAP" baseline 8.0 "$D/replay/combinedfull_baseline.json"
