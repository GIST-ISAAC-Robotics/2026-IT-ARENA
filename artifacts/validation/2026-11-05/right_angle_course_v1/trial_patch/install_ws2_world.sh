#!/usr/bin/env bash
# ws2(격리 시험 후보)의 실행 월드만 교체. 사용: install_ws2_world.sh <월드 디렉터리 이름>
set -euo pipefail
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
CW="$D/${1:?world}"
W="$HOME/arena_scratch/right_angle_v1/ws2/src/arena_gazebo/worlds/it_arena_official"
for f in world.sdf scene.json centerline.csv provenance.json; do cp "$CW/$f" "$W/$f"; done
cp "$CW/aruco/"*.png "$W/aruco/"
for f in world.sdf scene.json centerline.csv; do
  echo "ws2_world_$f $(sha256sum < "$W/$f" | cut -c1-64) expected $(sha256sum < "$CW/$f" | cut -c1-64)"
done
