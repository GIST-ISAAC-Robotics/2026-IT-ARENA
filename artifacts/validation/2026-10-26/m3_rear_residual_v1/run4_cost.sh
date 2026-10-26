#!/usr/bin/env bash
# 유용한 동작(해제)이 확인된 후보만 단일 프로세스·순차로 비용을 잰다. 다른 측정 작업이 없는 상태에서 실행한다.
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
D=artifacts/validation/2026-10-26/m3_rear_residual_v1
O="$D/run4_cost"
[ ! -e "$O" ] || exit 91
uptime > "$D/run4_load_before.txt"
python3 -I -B "$D/compare_release_premises.py" --plan "$D/run4_plan.json" --workers 1 --output "$O" > "$D/run4_stdout.txt" 2> "$D/run4_stderr.txt"
code=$?
uptime > "$D/run4_load_after.txt"
printf '%s\n' "$code" > "$D/run4_exit_code.txt"
exit "$code"
