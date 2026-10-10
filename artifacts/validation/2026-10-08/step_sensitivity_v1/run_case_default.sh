#!/usr/bin/env bash
# 현재 소스·기본 1 ms(step 인자 없음) 고정 요청 제동 시험 1회. 인자: <legacy|passive_damping> <flat|official> <새 실행명>
# 실행 전 시험 잔존 프로세스가 있으면 강제 종료하지 않고 중단한다. 기존 이름은 재사용하지 않는다.
set -e
source "$(dirname "$0")/env.sh"
case "$1" in legacy|passive_damping) mode="$1";; *) exit 2;; esac
case "$2" in official|flat) surface="$2";; *) exit 2;; esac
name="${3:?run name}"
case "$name" in *[!a-zA-Z0-9_]*) exit 2;; esac
test ! -e "$base/$name"
test ! -e "$base/$name.runner.log"
residual=$(ps -eo pid,cmd | grep -E 'gz sim|probe_braking_contact|validate_g1_traffic|parameter_bridge|rotating_lidar|actuation_ros|local_pursuit|lidar_safety' | grep -v grep || true)
if [ -n "$residual" ]; then echo "RESIDUAL_BEFORE $name"; echo "$residual"; exit 95; fi
set +e
python3 -B scripts/probe_braking_contact.py --surface "$surface" --zero-speed-mode "$mode" \
  --command-profile artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json \
  --output "$base/$name" > "$base/$name.runner.log" 2>&1
rc=$?
residual=$(ps -eo pid,cmd | grep -E 'gz sim|probe_braking_contact|parameter_bridge|rotating_lidar|actuation_ros|lidar_safety' | grep -v grep || true)
echo "RUN $name mode=$mode surface=$surface step=default rc=$rc residual_after=$([ -n "$residual" ] && echo yes || echo no)"
python3 -c "import json;r=json.load(open('$base/$name/report.json'));print(json.dumps({k:r.get(k) for k in ('passed','input_valid','stopped','shutdown_clean','stop_acknowledged','source_unchanged','error','wall_elapsed_s','arm_done_wall_s','finish_wall_s','finish_sim_s')}))" || echo '{"report":"missing"}'
exit $rc
