#!/usr/bin/env bash
# arena_autonomy만 기존 방식(--symlink-install)으로 빌드하고 설치 진입점을 확인한다. Gazebo 실행 없음.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
base=artifacts/validation/2026-10-29/g2_entry_gazebo_v1
test ! -e "$base/build1.log" || exit 90
colcon build --packages-select arena_autonomy --symlink-install --event-handlers console_direct+ > "$base/build1.log" 2>&1
result=$?
source install/local_setup.bash || exit 92
{
  echo "build_exit=$result"
  ls -l install/arena_autonomy/lib/arena_autonomy/
  echo "--- g2 entry script"
  cat install/arena_autonomy/lib/arena_autonomy/g2_entry_trial
  echo "--- ros2 pkg executables"
  ros2 pkg executables arena_autonomy
  echo "--- import"
  /usr/bin/python3 -B -c "import arena_autonomy.g2_entry_node as m, arena_autonomy, pathlib; print(pathlib.Path(m.__file__).resolve()); print(m.G2EntryNode.__mro__[1].__name__)"
  echo "--- default launch files do not name g2"
  grep -l g2_entry_trial src/arena_bringup/launch/*.py || echo none
} > "$base/build1_entry.log" 2>&1
printf '{"build":%s}\n' "$result" > "$base/build1_exit.json"
exit "$result"
