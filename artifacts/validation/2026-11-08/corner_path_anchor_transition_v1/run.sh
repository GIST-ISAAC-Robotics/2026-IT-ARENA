#!/usr/bin/env bash
# 사용: wsl.exe -e bash <this> <script> [args...]   (LF 전용. 실제 명령과 종료 코드를 기록한다. Gazebo 미사용)
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/corner_path_anchor_transition_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
N=${1:?script}; shift
TAG=$N${1:+_$1}
CMD=(bash "$ENV" /usr/bin/python3 -B "$A/$N.py" "$@")
start=$(date +%s)
"${CMD[@]}" > "$A/${TAG}_v1.log" 2>&1
code=$?
echo "COMMAND=${CMD[*]} EXIT=$code WALL_S=$(( $(date +%s)-start ))" > "$A/${TAG}_v1.exit.txt"
exit "$code"
