#!/usr/bin/env bash
# 실제 launch 선택 확인 1회: track:=official_v2026_11_06 + 기존 traffic_light_controller(평가 전용 재질 전환).
# 자율주행·제어기 노드 없음(autonomy 기본 false). 차량은 그리드에 정지 생성만 된다.
# 사용: launch_probe.sh <출력 이름>
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-07/track_migration_v1
NAME=${1:-launch_probe_v1}
OUT="$A/$NAME"
mkdir "$OUT" || exit 3
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export GZ_PARTITION="track_migration_$$" ROS_DOMAIN_ID=183 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$OUT/preflight_processes.txt"
if [ -s "$OUT/preflight_processes.txt" ]; then echo "ACTUAL_EXIT_CODE=4 preflight_busy" > "$OUT.exit"; exit 4; fi
setsid ros2 launch --noninteractive arena_bringup simulation.launch.py track:=official_v2026_11_06 headless:=true \
  render_backend:=wsl_nvidia traffic_light:=true red_duration_s:=2.0 grid_slot:=2 \
  runtime_artifacts:="$PWD/$OUT/evidence" > "$OUT/launch.log" 2>&1 < /dev/null &
pid=$!
start=$(date +%s); reason=timeout
while kill -0 "$pid" 2>/dev/null; do
  if grep -q '"stage": "green", "applied_sim_s"' "$OUT/evidence/signal_sequence.jsonl" 2>/dev/null; then reason=green_applied; sleep 3; break; fi
  if grep -q '"failed_stage"' "$OUT/evidence/signal_sequence.jsonl" 2>/dev/null; then reason=lamp_failure; break; fi
  [ $(( $(date +%s) - start )) -ge 240 ] && break
  sleep 1
done
kill -0 "$pid" 2>/dev/null || reason="launch_exited_early:$reason"
kill -INT -- -"$pid" 2>/dev/null
for i in $(seq 1 40); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
kill -0 "$pid" 2>/dev/null && { kill -TERM -- -"$pid" 2>/dev/null; sleep 5; kill -KILL -- -"$pid" 2>/dev/null; reason="$reason;forced"; }
wait "$pid"; launch_code=$?
sleep 2
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$OUT/postflight_processes.txt"
{
  echo "stop_reason=$reason launch_exit=$launch_code elapsed_s=$(( $(date +%s) - start ))"
  sha256sum "$OUT/evidence/runtime_world.sdf" src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf src/arena_gazebo/worlds/it_arena_official/world.sdf
  grep -m1 'Track: ' "$OUT/launch.log"
  grep -c 'Err\]' "$OUT/launch.log"
} > "$OUT/summary.txt" 2>&1
ok=0
[ "$reason" = green_applied ] || ok=1
[ -s "$OUT/postflight_processes.txt" ] && ok=1
echo "ACTUAL_EXIT_CODE=$ok" > "$OUT.exit"
exit "$ok"
