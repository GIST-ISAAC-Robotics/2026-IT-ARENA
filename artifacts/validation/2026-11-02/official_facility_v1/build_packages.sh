#!/usr/bin/env bash
# 월드 재생성 → 패키지 빌드 → 재질 클라이언트 자체 검사. 실패하면 이후 단계를 건너뛴다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_facility_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
{
  python3 -B scripts/build_official_track.py > "$A/build_world.json" &&
  python3 -B scripts/build_official_track.py --check &&
  colcon build --packages-select arena_gazebo arena_bringup arena_autonomy --symlink-install &&
  source install/local_setup.bash &&
  GZ_PARTITION=arena_material_selftest_local "$(ros2 pkg prefix arena_gazebo)/lib/arena_gazebo/visual_material_client" --self-test
} > "$A/build_packages.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/build_packages.exit"
exit "$code"
