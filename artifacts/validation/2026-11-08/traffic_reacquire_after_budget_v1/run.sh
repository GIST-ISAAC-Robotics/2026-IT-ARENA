#!/usr/bin/env bash
# 사용: wsl.exe -e bash <this> <replay|probe>   (LF 전용. 실제 명령과 종료 코드를 기록한다. Gazebo 미사용)
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/traffic_reacquire_after_budget_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
N=${1:?replay|probe}
CMD=(bash "$ENV" /usr/bin/python3 -B "$A/$N.py")
start=$(date +%s)
"${CMD[@]}" > "$A/${N}_v1.log" 2>&1
code=$?
echo "COMMAND=${CMD[*]} EXIT=$code WALL_S=$(( $(date +%s)-start ))" > "$A/${N}_v1.exit.txt"
exit "$code"
