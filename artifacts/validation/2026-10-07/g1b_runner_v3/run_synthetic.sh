#!/usr/bin/env bash
# 최종 소스로 G1-a 합성 폐루프 재확인(Gazebo 아님). 출력 새 폴더.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out=artifacts/validation/2026-10-07/g1b_resume_synthetic_v1
test ! -e "$out" || { echo "output exists"; exit 93; }
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -B scripts/validate_local_follow.py --output "$out" > artifacts/validation/2026-10-07/g1b_runner_v3/logs/g1b_resume_synthetic_v1.log 2>&1
rc=$?
echo "$rc" > artifacts/validation/2026-10-07/g1b_runner_v3/logs/g1b_resume_synthetic_v1.exitcode
tail -5 artifacts/validation/2026-10-07/g1b_runner_v3/logs/g1b_resume_synthetic_v1.log
echo "rc=$rc"
