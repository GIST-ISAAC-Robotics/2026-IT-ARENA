#!/usr/bin/env bash
# 사용: render_scene.sh <clip_name> <run_dir_relative_to_repo> <t0> <t1>
# 기록 자세 재렌더 1회. 결과: ~/arena_scratch/feature_showcase/clips/<clip_name>/
set -eo pipefail
CLIP="$1"; RUN_REL="$2"; T0="$3"; T1="$4"
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
TOOLS="$REPO/artifacts/videos/2026-11-09/feature_showcase/tools"
W="$HOME/arena_scratch/feature_showcase"
OUT="$W/clips/$CLIP"
PLUG="$W/build"
rm -rf "$OUT"; mkdir -p "$OUT"
cp -f "$REPO/install/arena_gazebo/lib/libarena-wsl-d3d12-lifetime.so" "$PLUG/"
python3 -I "$TOOLS/build_replay_scene.py" --run "$REPO/$RUN_REL" --t0 "$T0" --t1 "$T1" --out "$OUT" --plugin-dir "$PLUG" > "$OUT/build.log"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export GALLIUM_DRIVER=d3d12 LIBGL_ALWAYS_SOFTWARE=false MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export LD_LIBRARY_PATH="$PLUG:${LD_LIBRARY_PATH:-}" LD_PRELOAD=libarena-wsl-d3d12-lifetime.so
export GZ_PARTITION="showcase_$CLIP"
gz sim -s -r -v 3 --headless-rendering "$OUT/render_world.sdf" > "$OUT/gz.log" 2>&1 &
GZ_PID=$!
START=$(date +%s)
while [ ! -f "$OUT/done.txt" ]; do
  if ! kill -0 "$GZ_PID" 2>/dev/null; then echo "gz exited early" >> "$OUT/gz.log"; break; fi
  if [ $(( $(date +%s) - START )) -gt 900 ]; then echo "wall timeout" >> "$OUT/gz.log"; break; fi
  sleep 1
done
kill -INT "$GZ_PID" 2>/dev/null || true
for i in $(seq 1 20); do kill -0 "$GZ_PID" 2>/dev/null || break; sleep 0.5; done
kill -KILL "$GZ_PID" 2>/dev/null || true
pkill -KILL -f "^gz sim -s -r -v 3 --headless-rendering $OUT" 2>/dev/null || true
wait "$GZ_PID" 2>/dev/null || true
END=$(date +%s)
N=$(ls "$OUT/frames" | wc -l)
echo "clip=$CLIP frames=$N wall_s=$((END-START)) done=$(cat "$OUT/done.txt" 2>/dev/null)" | tee "$OUT/render_result.txt"
grep -iE "GL_RENDERER|error|PoseReplay" "$OUT/gz.log" | head -10 || true
