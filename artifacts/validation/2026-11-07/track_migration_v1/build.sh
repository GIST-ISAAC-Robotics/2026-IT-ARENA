#!/usr/bin/env bash
# v2026.11.06 이관: (1) 현재 빌더로 v2026.11.01 재현 대조 (2) 새 월드 생성 (3) 두 월드 check (4) 옛 world.sdf 해시.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-07/track_migration_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
step() { name=$1; shift; "$@" > "$A/checks/$name.log" 2>&1; code=$?; echo "ACTUAL_EXIT_CODE=$code" > "$A/checks/$name.exit"; echo "$name exit=$code"; return $code; }
step rebuild_v2026_11_01 python3 -B "$A/rebuild_compare.py" || exit 1
step build_v2026_11_06 python3 -B scripts/build_official_track.py --profile config/tracks/official_v2026.11.06.yaml || exit 1
step check_v2026_11_06 python3 -B scripts/build_official_track.py --check --profile config/tracks/official_v2026.11.06.yaml || exit 1
step check_v2026_11_01 python3 -B scripts/build_official_track.py --check || exit 1
sha256sum src/arena_gazebo/worlds/it_arena_official/world.sdf | tee "$A/checks/old_world_sha256.txt"
grep -q '^e4fe4d0d2fa13a1a60898e15a5b21cb36a3080835319dda09ccb97ae65c9e048 ' "$A/checks/old_world_sha256.txt"
