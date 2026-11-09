#!/usr/bin/env bash
# 정지 사진 전용 렌더; 기존 주행/원본/영상 산출물은 변경하지 않는다.
set -eo pipefail
BASE="$1"
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
PLUG="$HOME/arena_scratch/feature_showcase/build"
if pgrep -af '^gz sim' > /dev/null; then echo 'preexisting gz sim; refuse to interfere'; exit 3; fi
test -f "$PLUG/libshowcase-pose-replay.so"
source /opt/ros/jazzy/setup.bash
export GALLIUM_DRIVER=d3d12 LIBGL_ALWAYS_SOFTWARE=false MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export LD_LIBRARY_PATH="$REPO/install/arena_gazebo/lib:$PLUG:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD=libarena-wsl-d3d12-lifetime.so
for NAME in racing solo original; do
  if [ -n "${ONLY:-}" ] && [[ " $ONLY " != *" $NAME "* ]]; then continue; fi
  OUT="$BASE/$NAME"
  test -f "$OUT/render_world.sdf"
  export GZ_PARTITION="poster_closeup_${NAME}_$$"
  gz sim -s -r -v 3 --headless-rendering "$OUT/render_world.sdf" > "$OUT/gz.log" 2>&1 &
  PID=$!
  START=$(date +%s)
  while [ ! -f "$OUT/done.txt" ]; do
    kill -0 "$PID" 2>/dev/null || break
    if [ $(( $(date +%s)-START )) -gt 180 ]; then echo 'render wall timeout'; break; fi
    sleep 1
  done
  kill -INT "$PID" 2>/dev/null || true
  for i in $(seq 1 20); do kill -0 "$PID" 2>/dev/null || break; sleep .5; done
  if kill -0 "$PID" 2>/dev/null; then kill -KILL "$PID" 2>/dev/null || true; fi
  wait "$PID" 2>/dev/null || true
  test -f "$OUT/done.txt"
  echo "$NAME $(cat "$OUT/done.txt")"
done
