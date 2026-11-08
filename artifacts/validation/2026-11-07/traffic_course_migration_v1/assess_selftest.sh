#!/usr/bin/env bash
# assess.py를 기존 r14 run1(옛 코스·B0183)에 임시 폴더로 연결해 오류 없이 도는지만 확인한다. 결과는 판정 근거가 아니다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-07/traffic_course_migration_v1
W=artifacts/validation/2026-11-05/moving_pass_withdrawal_v1/candidate_r14
T=$(mktemp -d)
mkdir "$T/straight_moving_lead"
ln -s "$PWD/$W/run1" "$T/straight_moving_lead/run1"
cp "$W/surface.json" "$W/registered.json" "$T/straight_moving_lead/"
echo '{"changed": []}' > "$T/straight_moving_lead/postflight_integrity.json"
: > "$T/straight_moving_lead/postflight_processes.txt"
cp "$A/new_track_course.json" "$T/"
set +u; source /opt/ros/jazzy/setup.bash
PYTHONPATH="$A" python3 -B -c "
import assess; from pathlib import Path
assess.HERE = Path('$T'); raise SystemExit(assess.main('straight_moving_lead'))" > "$A/checks/assess_selftest.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/checks/assess_selftest.exit"
rm -rf "$T"
exit "$code"
