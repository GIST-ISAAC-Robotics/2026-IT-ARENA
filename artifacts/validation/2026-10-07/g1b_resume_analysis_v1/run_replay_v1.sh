#!/usr/bin/env bash
# 보존 등속 실패 자료를 이전/현재 LocalFollow로 open-loop 재생한다(분석 전용).
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-10-07/g1b_resume_analysis_v1
R=artifacts/validation/2026-10-07/g1b_pilot_v3_slow_s0
if [ ! -d "$A/source_before/src" ]; then
  # 재개 시작 소스 = HEAD의 추적 파일 + 시작 상태 ZIP의 dirty/미추적 파일(로컬 재생성용, Git 제외).
  mkdir -p "$A/source_before"
  git archive HEAD src/arena_autonomy src/arena_vehicle_interface | tar -x -C "$A/source_before"
  python3 -c "import zipfile;zipfile.ZipFile('artifacts/validation/2026-10-07/g1b_resume_start_v1/source_before.zip').extractall('$A/source_before')"
fi
for g in zero evaluation_pose_rate; do
  echo "== old $g"; python3 -B scripts/replay_g1b_follow.py $R --first 3.5 --last 16.0 --gyro $g --source-root $A/source_before --output $A/replay_old_$g.json
  echo "== new $g"; python3 -B scripts/replay_g1b_follow.py $R --first 3.5 --last 16.0 --gyro $g --output $A/replay_new_$g.json
done
echo "== old cold 9.0"; python3 -B scripts/replay_g1b_follow.py $R --first 8.5 --last 16.0 --cold-start 9.0 --source-root $A/source_before --output $A/replay_old_cold9.json
echo "== new cold 9.0"; python3 -B scripts/replay_g1b_follow.py $R --first 8.5 --last 16.0 --cold-start 9.0 --output $A/replay_new_cold9.json
