#!/usr/bin/env bash
# R2~R6 등록 순서 순차 실행. 한 실행이라도 0이 아니면 즉시 멈춘다(재시도는 하지 않음).
set -u
dir="$(dirname "$0")"
for spec in "0.00025 10 o025_p10_a" "0.001 0 o1ms_p0" "0.00025 0 o025_p0" "0.001 10 o1ms_p10_b" "0.00025 10 o025_p10_b"; do
  set -- $spec
  bash "$dir/run_case.sh" "$1" "$2" "$3"
  rc=$?
  echo "BATCH_RESULT $3 rc=$rc"
  if [ $rc -ne 0 ]; then echo "BATCH_STOPPED_AT $3"; exit $rc; fi
done
echo BATCH_DONE
