#!/usr/bin/env bash
# [격리 시험 전용] ws(공용 트리와 같은 코드) → ws2 복사, 꺾은선 후보 적용, 빌드, 시험 월드 설치.
# 사용: setup_ws2.sh <월드 디렉터리 이름(course_world|iso_world)>
set -eo pipefail
MAIN="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="$MAIN/artifacts/validation/2026-11-05/right_angle_course_v1"
T="$A/trial_patch"
WS="$HOME/arena_scratch/right_angle_v1/ws"
W2="$HOME/arena_scratch/right_angle_v1/ws2"
CW="$A/${1:-course_world}"
if [ -e "$W2" ]; then echo "existing $W2; refuse" >&2; exit 4; fi
mkdir -p "$W2/artifacts"
for d in src scripts tests config; do
  rsync -a --exclude='__pycache__' --exclude='*.pyc' "$WS/$d/" "$W2/$d/"
done
/usr/bin/python3 -B "$T/apply_trial.py" "$W2"
diff -u "$WS/src/arena_autonomy/arena_autonomy/local_path.py" "$W2/src/arena_autonomy/arena_autonomy/local_path.py" > "$T/trial.patch" || true
diff -u "$WS/src/arena_autonomy/arena_autonomy/local_follow.py" "$W2/src/arena_autonomy/arena_autonomy/local_follow.py" >> "$T/trial.patch" || true
( cd "$W2" && find src scripts tests config -type f ! -path 'src/arena_gazebo/worlds/it_arena_official/*' | LC_ALL=C sort | xargs -d '\n' sha256sum ) > "$T/ws2_code_sha256.txt"
grep -v ' src/arena_gazebo/worlds/it_arena_official/' "$A/isolated_copy_sha256.txt" > "$T/ws_code_expected.txt"
diff "$T/ws_code_expected.txt" "$T/ws2_code_sha256.txt" > "$T/ws2_vs_main_code.diff" || true
rm -f "$T/ws_code_expected.txt"
cd "$W2"
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install > "$T/ws2_build.log" 2>&1
echo "BUILD_EXIT=$?"
W="$W2/src/arena_gazebo/worlds/it_arena_official"
for f in world.sdf scene.json centerline.csv provenance.json; do cp "$CW/$f" "$W/$f"; done
cp "$CW/aruco/"*.png "$W/aruco/"
for f in world.sdf scene.json centerline.csv; do
  echo "ws2_world_$f $(sha256sum < "$W/$f" | cut -c1-64) expected $(sha256sum < "$CW/$f" | cut -c1-64)"
done
echo "install_share_world=$(readlink -f "$W2/install/arena_gazebo/share/arena_gazebo/worlds/it_arena_official/world.sdf")"
echo "local_path_resolves=$(readlink -f "$W2/install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy/local_path.py" 2>/dev/null || echo n/a)"
