#!/usr/bin/env bash
# pilot 관문 확인용 R1 단독 분석(새 출력 이름만).
set -e
source "$(dirname "$0")/env.sh"
out="${1:?output}"; test ! -e "$base/$out"
python3 -B -m pytest -q -p no:cacheprovider tests/test_scheduled_phase_analysis.py 2>&1 | tail -2
python3 -B scripts/analyze_scheduled_phase.py --run R1="$base/o1ms_p10_a" --output "$base/$out"
