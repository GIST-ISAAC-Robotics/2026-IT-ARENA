#!/usr/bin/env bash
# 등록 계획 순차 실행. 인자: <plan.json> <새 status 이름>. 실패/잔존 시 즉시 멈춘다(재시도 없음).
# v1(run_batch.sh)은 ROS 설정 source 전에 set -u를 켜 시작 즉시 실패했다(사례 미시작). 보존하고 이 판을 쓴다.
source "$(dirname "$0")/env.sh"
set -u
plan="${1:?plan}"; name="${2:?status name}"
case "$name" in status_[a-z0-9_]*.json) ;; *) echo "bad status name"; exit 2;; esac
test ! -e "$base/$name" || { echo "status exists"; exit 2; }
python3 -B scripts/run_probe_batch.py --plan "$plan" --status "$base/$name" > "$base/${name%.json}.log" 2>&1
rc=$?
echo "BATCH_EXIT rc=$rc"
python3 -c "import json,sys;s=json.load(open(sys.argv[1]));print(json.dumps({'stopped_at':s['stopped_at'],'not_started':s['not_started'],'exit_code':s.get('exit_code'),'cases':[{k:c.get(k) for k in ('name','started','rc','residual_after')}|{'summary':{k:(c.get('summary') or {}).get(k) for k in ('passed','input_valid','stopped','shutdown_clean','error','max_lateness_ns','fault_injection_applied','plugin_trace_valid')}} for c in s['cases']]},indent=1))" "$base/$name"
ps -eo pid,args | grep -E 'gz sim|probe_braking_contact|parameter_bridge|rotating_lidar|actuation_ros|lidar_safety|gz topic -e' | grep -v grep || echo "NO_RESIDUAL"
exit $rc
