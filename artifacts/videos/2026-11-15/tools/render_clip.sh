#!/usr/bin/env bash
# 기록 자세 재렌더 1회(WSL). 사용: render_clip.sh <clip> <run_dir_relative_to_repo> <t0> <t1> [build_scene.py 추가 인자...]
# - 장면 생성 → 소유 gz sim 1개 실행 → done.txt 대기 → 소유 PID만 종료·확인 → PPM을 JPEG(q95)로 변환.
# - 다른 gz/ROS 프로세스가 있으면 건드리지 않고 중단한다(exit 3).
set -eo pipefail
CLIP="$1"; RUN_REL="$2"; T0="$3"; T1="$4"; shift 4
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
TOOLS="$REPO/artifacts/videos/2026-11-15/tools"
BASE="$REPO/build/competition_recordings_20261115"
PLUG="$BASE/plugin"
SCR="$HOME/arena_scratch/competition_recordings_20261115/$CLIP"
JPG="$BASE/frames/$CLIP"
LOGD="$BASE/logs/$CLIP"
mkdir -p "$LOGD"
PROC_LOG="$LOGD/processes.txt"
: > "$PROC_LOG"
echo "[preflight $(date -Is)]" >> "$PROC_LOG"
PRE=$(ps -eo pid,ppid,etimes,args | grep -E "gz sim|gz-sim|gzserver|ros2 |ruby .*gz" | grep -v -E "grep|render_clip.sh" || true)
echo "${PRE:-none}" >> "$PROC_LOG"
if [ -n "$PRE" ]; then echo "preexisting gz/ros process; abort" | tee -a "$PROC_LOG"; exit 3; fi

rm -rf "$SCR"; mkdir -p "$SCR"
cp -fL "$REPO/install/arena_gazebo/lib/libarena-wsl-d3d12-lifetime.so" "$PLUG/"
python3 -I "$TOOLS/build_scene.py" --run "$REPO/$RUN_REL" --t0 "$T0" --t1 "$T1" --out "$SCR" \
  --plugin "$PLUG/librecording-pose-replay.so" "$@" > "$LOGD/build_scene.log" 2>&1 || { cat "$LOGD/build_scene.log"; exit 1; }
cat "$LOGD/build_scene.log"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export GALLIUM_DRIVER=d3d12 LIBGL_ALWAYS_SOFTWARE=false MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export LD_LIBRARY_PATH="$PLUG:${LD_LIBRARY_PATH:-}" LD_PRELOAD=libarena-wsl-d3d12-lifetime.so
export GZ_PARTITION="rec20261115_${CLIP}_$$" GZ_IP=127.0.0.1
START=$(date +%s.%N)
gz sim -s -r -v 3 --headless-rendering "$SCR/render_world.sdf" > "$LOGD/gz.log" 2>&1 &
GZ_PID=$!
echo "[launched $(date -Is)] gz_pid=$GZ_PID partition=$GZ_PARTITION" >> "$PROC_LOG"
sleep 3
ps -eo pid,ppid,args | awk -v p="$GZ_PID" '$1==p || $2==p' >> "$PROC_LOG" || true
STATUS=ok
while [ ! -f "$SCR/done.txt" ]; do
  if ! kill -0 "$GZ_PID" 2>/dev/null; then STATUS=gz_exited_early; break; fi
  if awk -v s="$START" -v n="$(date +%s.%N)" 'BEGIN{exit !(n-s>1800)}'; then STATUS=wall_timeout; break; fi
  sleep 1
done
END=$(date +%s.%N)
# 소유 프로세스 트리(gz PID와 그 자식)만 종료한다.
KIDS=$(pgrep -P "$GZ_PID" || true)
kill -INT "$GZ_PID" 2>/dev/null || true
for i in $(seq 1 30); do kill -0 "$GZ_PID" 2>/dev/null || break; sleep 0.5; done
FORCED=no
if kill -0 "$GZ_PID" 2>/dev/null; then kill -KILL "$GZ_PID" 2>/dev/null || true; FORCED=yes; fi
for k in $KIDS; do kill -0 "$k" 2>/dev/null && { kill -KILL "$k" 2>/dev/null || true; FORCED=yes; }; done
wait "$GZ_PID" 2>/dev/null || true
sleep 1
LEFT=$(ps -eo pid,args | grep -F -- "$SCR/render_world.sdf" | grep -v grep || true)
LEFT_KIDS=""
for k in $KIDS; do kill -0 "$k" 2>/dev/null && LEFT_KIDS="$LEFT_KIDS $k"; done
ALIVE=no; kill -0 "$GZ_PID" 2>/dev/null && ALIVE=yes
echo "[after $(date -Is)] status=$STATUS gz_pid=$GZ_PID children=[$(echo $KIDS)] forced_kill=$FORCED gz_alive=$ALIVE residual_children=[${LEFT_KIDS}] residual_world_procs=[${LEFT:-none}]" | tee -a "$PROC_LOG"
POST=$(ps -eo pid,ppid,etimes,args | grep -E "gz sim|gz-sim|gzserver|ros2 |ruby .*gz" | grep -v -E "grep|render_clip.sh" || true)
echo "post_scan: ${POST:-none}" >> "$PROC_LOG"
cp -f "$SCR/meta.json" "$SCR/poses.txt" "$SCR/render_world.sdf" "$LOGD/" 2>/dev/null || true
cp -f "$SCR/done.txt" "$LOGD/" 2>/dev/null || true
rm -rf "$JPG"; mkdir -p "$JPG"
python3 -I "$TOOLS/convert_frames.py" "$SCR" "$JPG" >> "$LOGD/convert.log" 2>&1 || STATUS="${STATUS}_convert_failed"
WALL=$(awk -v s="$START" -v e="$END" 'BEGIN{printf "%.1f", e-s}')
echo "clip=$CLIP status=$STATUS wall_render_s=$WALL done=[$(cat "$SCR/done.txt" 2>/dev/null)] $(tail -1 "$LOGD/convert.log")" | tee "$LOGD/render_result.txt"
grep -iE "GL_RENDERER|OpenGL|error|RecordingReplay|anti" "$LOGD/gz.log" | head -12 || true
[ "$STATUS" = ok ] && [ "$ALIVE" = no ] && [ -z "$LEFT_KIDS" ] && [ -z "$LEFT" ]
