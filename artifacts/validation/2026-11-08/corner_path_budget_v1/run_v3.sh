#!/usr/bin/env bash
# 사용: wsl.exe -e bash -lc 'bash "<repo>/artifacts/validation/2026-11-08/corner_path_budget_v1/run_v3.sh" <name> <label>'  (LF. 명령·종료 코드·소요 시간 기록. Gazebo 미사용)
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/corner_path_budget_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
N=${1:?name}; L=${2:?label}
CMD=(bash "$ENV" /usr/bin/python3 -B "$A/$N.py")
start=$(date +%s)
"${CMD[@]}" > "$A/${N}_${L}.log" 2>&1
code=$?
echo "COMMAND=${CMD[*]} EXIT=$code WALL_S=$(( $(date +%s)-start ))" > "$A/${N}_${L}.exit.txt"
exit "$code"
