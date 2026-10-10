#!/usr/bin/env bash
# 격자 발행 고정 요청 1회(분할 노면 official, passive_damping 고정).
# 인자: <0.001|0.00025> <0|10> <새 실행명>. 0.001은 월드 기본 step(--physics-step 생략).
# 실행 전 시험 잔존 프로세스가 있으면 강제 종료하지 않고 중단한다. 기존 이름은 재사용하지 않는다.
set -e
source "$(dirname "$0")/env.sh"
case "$1" in 0.001) step_args=();; 0.00025) step_args=(--physics-step 0.00025);; *) exit 2;; esac
case "$2" in 0|10) phase="$2";; *) exit 2;; esac
name="${3:?run name}"
case "$name" in *[!a-zA-Z0-9_]*) exit 2;; esac
test ! -e "$base/$name"
test ! -e "$base/$name.runner.log"
residual=$(ps -eo pid,cmd | grep -E 'gz sim|probe_braking_contact|validate_g1_traffic|parameter_bridge|rotating_lidar|actuation_ros|local_pursuit|lidar_safety' | grep -v grep || true)
if [ -n "$residual" ]; then echo "RESIDUAL_BEFORE $name"; echo "$residual"; exit 95; fi
set +e
python3 -B scripts/probe_braking_contact.py --surface official --zero-speed-mode passive_damping "${step_args[@]}" \
  --command-schedule sim_grid --schedule-phase-ms "$phase" \
  --command-profile artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json \
  --output "$base/$name" > "$base/$name.runner.log" 2>&1
rc=$?
residual=$(ps -eo pid,cmd | grep -E 'gz sim|probe_braking_contact|parameter_bridge|rotating_lidar|actuation_ros|lidar_safety' | grep -v grep || true)
echo "RUN $name step=$1 phase=$phase rc=$rc residual_after=$([ -n "$residual" ] && echo yes || echo no)"
python3 -c "import json;r=json.load(open('$base/$name/report.json'));a=r.get('request_audit') or {};print(json.dumps({k:r.get(k) for k in ('passed','input_valid','stopped','shutdown_clean','stop_acknowledged','source_unchanged','error','wall_elapsed_s')}|{'audit':a.get('checks'),'lateness':a.get('lateness_ns'),'clock':(r.get('clock_observation') or {}).get('delta_ns_top')}))" || echo '{"report":"missing"}'
exit $rc
