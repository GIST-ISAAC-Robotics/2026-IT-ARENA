#!/usr/bin/env bash
# 플러그인 계측 빌드·CTest·바이너리 해시. 로그 이름은 새 이름만 허용한다.
set -e
source "$(dirname "$0")/env.sh"
tag="${1:?tag}"
test ! -e "$base/build_$tag.log"
colcon build --packages-select arena_gazebo arena_bringup --symlink-install > "$base/build_$tag.log" 2>&1
ctest --test-dir build/arena_gazebo --output-on-failure > "$base/ctest_$tag.log" 2>&1
sha256sum install/arena_gazebo/lib/libarena-single-motor-system.so > "$base/plugin_$tag.sha256"
cat "$base/plugin_$tag.sha256"; tail -3 "$base/build_$tag.log"; tail -3 "$base/ctest_$tag.log"
