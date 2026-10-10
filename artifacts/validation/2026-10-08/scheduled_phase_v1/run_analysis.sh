#!/usr/bin/env bash
# 등록 행렬 분석. 인자: 새 출력 이름(.json). 재시도 실행을 쓰면 해당 이름을 R 변수로 바꿔 호출한다.
set -e
source "$(dirname "$0")/env.sh"
out="${1:?output}"; test ! -e "$base/$out"
R1="${R1:-o1ms_p10_a}"; R2="${R2:-o025_p10_a}"; R3="${R3:-o1ms_p0}"; R4="${R4:-o025_p0}"; R5="${R5:-o1ms_p10_b}"; R6="${R6:-o025_p10_b}"
python3 -B scripts/analyze_scheduled_phase.py \
  --run R1="$base/$R1" --run R2="$base/$R2" --run R3="$base/$R3" --run R4="$base/$R4" --run R5="$base/$R5" --run R6="$base/$R6" \
  --pair repeat:R1:R5:1ms_phase10 --pair repeat:R2:R6:0p25ms_phase10 \
  --pair phase:R1:R3:1ms --pair phase:R2:R4:0p25ms \
  --pair dt:R1:R2:a --pair dt:R3:R4:p0 --pair dt:R5:R6:b \
  --output "$base/$out" > "$base/${out%.json}.log" 2>&1
rc=$?
cat "$base/${out%.json}.log"
exit $rc
