#!/usr/bin/env bash
# 전체 재현: 1단계 센서 관측(r11, r27) → 2단계 사후 평가 → 사후 보조 점검. 새 시뮬레이션 없음.
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 2
bash run_observe.sh r11 || exit $?
bash run_observe.sh r27 || exit $?
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -I -B evaluate.py > evaluate.log 2>&1
code=$?
echo "$code" > evaluate.exit
[ "$code" = 0 ] || exit "$code"
bash run_sensitivity.sh || exit $?
python3 -I -B crossing.py > crossing.log 2>&1
code=$?
echo "$code" > crossing.exit
exit "$code"
