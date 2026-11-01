#!/usr/bin/env bash
# 빌드 → 사전 점검 → 등록 입력 대조 → 단독 1회. 실패해도 재실행하지 않는다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-01/solo_ray_join_v1
# 대기 명령 문자열에 맞지 않도록 실제 실행 파일로 시작하는 프로세스만 본다.
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight.log"; then
  echo "OTHER_SIMULATION_RUNNING" >> "$A/preflight.log"
  echo "ACTUAL_EXIT_CODE=3" > "$A/run_once.exit"
  exit 3
fi
echo "NO_OTHER_SIMULATION" >> "$A/preflight.log"
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
python3 -B "$A/run.py"
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/run_once.exit"
exit "$code"
