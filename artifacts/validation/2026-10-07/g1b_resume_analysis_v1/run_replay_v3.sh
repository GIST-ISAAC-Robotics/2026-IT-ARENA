#!/usr/bin/env bash
# 2차 수정(기억이 있을 때도 반대 벽 기준 재선택) 뒤 같은 보존 자료 재생(분석 전용).
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-10-07/g1b_resume_analysis_v1
python3 -B scripts/replay_g1b_follow.py artifacts/validation/2026-10-07/g1b_resume_v1_slow_s0 --first 3.5 --last 16.4 --output $A/replay_final_resumev1_zero.json
python3 -B scripts/replay_g1b_follow.py artifacts/validation/2026-10-07/g1b_pilot_v3_slow_s0 --first 3.5 --last 16.0 --output $A/replay_final_pilotv3_zero.json
python3 -B scripts/replay_g1b_follow.py artifacts/validation/2026-10-07/g1b_pilot_v3_slow_s0 --first 3.5 --last 16.0 --gyro evaluation_pose_rate --output $A/replay_final_pilotv3_evaluation_pose_rate.json
python3 -B scripts/replay_g1b_follow.py artifacts/validation/2026-10-07/g1b_pilot_v3_slow_s0 --first 8.5 --last 16.0 --cold-start 9.0 --output $A/replay_final_pilotv3_cold9.json
python3 -B scripts/replay_g1b_follow.py artifacts/validation/2026-10-07/g1b_pilot_v3_stopped_s0 --first 3.5 --last 14.0 --output $A/replay_final_stoppedv3_zero.json
python3 -B scripts/replay_g1b_follow.py artifacts/validation/2026-10-07/g1b_pilot_v3_stopped_s0 --first 3.5 --last 14.0 --source-root $A/source_before --output $A/replay_old_stoppedv3_zero.json
