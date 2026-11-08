#!/usr/bin/env bash
# 새 월드 디렉터리 설치와 launch 변경 반영. 다른 패키지는 빌드하지 않는다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-07/track_migration_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
colcon build --packages-select arena_gazebo arena_bringup --symlink-install > "$A/checks/colcon.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/checks/colcon.exit"
set +u; source install/local_setup.bash
share=$(ros2 pkg prefix arena_gazebo)/share/arena_gazebo/worlds/it_arena_official_v2026_11_06
{ ls -la "$share/world.sdf" "$share/scene.json"; sha256sum "$share/world.sdf" src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf; } >> "$A/checks/colcon.log" 2>&1
exit "$code"
