#!/usr/bin/env bash
# 직각 코너 시험용 격리 작업공간. 공용 작업트리(src/scripts/tests/config)의 현재 바이트를
# WSL 로컬 디스크로 복사해 따로 빌드한다. 공용 트리·install·공식 월드는 수정하지 않는다.
set -eo pipefail
MAIN="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
WS="$HOME/arena_scratch/right_angle_v1/ws"
A="$MAIN/artifacts/validation/2026-11-05/right_angle_course_v1"
if [ -e "$WS" ]; then echo "existing $WS; refuse" >&2; exit 4; fi
mkdir -p "$WS"
for d in src scripts tests config; do
  rsync -a --exclude='__pycache__' --exclude='*.pyc' "$MAIN/$d/" "$WS/$d/"
done
mkdir -p "$WS/artifacts"
# 복사 직후 원본과 같은지 해시로 기록한다(월드 교체 전).
( cd "$WS" && find src scripts tests config -type f | LC_ALL=C sort | xargs -d '\n' sha256sum ) > "$A/isolated_copy_sha256.txt"
( cd "$MAIN" && find src scripts tests config -type f ! -path '*/__pycache__/*' ! -name '*.pyc' | LC_ALL=C sort | xargs -d '\n' sha256sum ) > "$A/main_tree_sha256_at_copy.txt"
if cmp -s "$A/isolated_copy_sha256.txt" "$A/main_tree_sha256_at_copy.txt"; then echo COPY_IDENTICAL=1; else echo COPY_IDENTICAL=0; fi
cd "$WS"
set +u; source /opt/ros/jazzy/setup.bash; set -u
start=$(date +%s)
colcon build --symlink-install > "$A/isolated_build.log" 2>&1
echo "BUILD_EXIT=$? WALL_S=$(( $(date +%s) - start ))"
