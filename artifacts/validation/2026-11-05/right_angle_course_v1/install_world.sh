#!/usr/bin/env bash
# 격리 사본(WSL ~/arena_scratch/right_angle_v1/ws)의 it_arena_official 실행 파일만 시험 월드로 바꾼다.
# 공용 작업트리·공용 install·공식 원본은 건드리지 않는다. 사용하지 않는 공식 부속 파일은 그대로 두고 기록한다.
set -euo pipefail
MAIN="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="$MAIN/artifacts/validation/2026-11-05/right_angle_course_v1"
WS="$HOME/arena_scratch/right_angle_v1/ws"
W="$WS/src/arena_gazebo/worlds/it_arena_official"
CW="$A/${1:-course_world}"
case "$(readlink -f "$W")" in /mnt/*) echo "refuse: world dir is on the shared tree" >&2; exit 4;; esac
for f in world.sdf scene.json centerline.csv provenance.json; do
  cp "$CW/$f" "$W/$f"
done
cp "$CW/aruco/"*.png "$W/aruco/"
{
  echo "world_dir=$CW"
  echo "installed_utc=$(date -u +%FT%TZ)"
  for f in world.sdf scene.json centerline.csv provenance.json; do
    echo "$f copy=$(sha256sum < "$W/$f" | cut -c1-64) source=$(sha256sum < "$CW/$f" | cut -c1-64)"
  done
  echo "install_share_world_resolves_to=$(readlink -f "$WS/install/arena_gazebo/share/arena_gazebo/worlds/it_arena_official/world.sdf")"
  echo "install_share_scene_resolves_to=$(readlink -f "$WS/install/arena_gazebo/share/arena_gazebo/worlds/it_arena_official/scene.json")"
  echo "unused_official_leftovers_in_copy: $(cd "$W" && ls | grep -v -E '^(world.sdf|scene.json|centerline.csv|provenance.json|aruco)$' | tr '\n' ' ')"
  echo "main_official_world_sha256=$(sha256sum < "$MAIN/src/arena_gazebo/worlds/it_arena_official/world.sdf" | cut -c1-64)"
} >> "$A/install_world.log"
tail -12 "$A/install_world.log"
