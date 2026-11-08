#!/usr/bin/env bash
# 저장 기록 재평가(새 주행 아님). 1) r48 대조로 방법 확인 2) run1 재평가 3) 보충 판정 재집계
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/grid_staggered_start_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
mkdir -p "$A/reeval"
bash "$ENV" python3 -B "$A/reevaluate.py" control > "$A/reeval/control.log" 2>&1
c=$?; printf '%s\n' "$c" > "$A/reeval/control.exit"; [ "$c" -eq 0 ] || exit 5
bash "$ENV" python3 -B "$A/reevaluate.py" run1 > "$A/reeval/reevaluate.log" 2>&1
r=$?; printf '%s\n' "$r" > "$A/reeval/reevaluate.exit"; [ "$r" -eq 0 ] || exit 6
bash "$ENV" python3 -B "$A/reassess.py" > "$A/reeval/assessment.log" 2>&1
s=$?; printf '%s\n' "$s" > "$A/reeval/assessment.exit"
echo "CONTROL=$c REEVALUATE=$r ASSESSMENT=$s"
