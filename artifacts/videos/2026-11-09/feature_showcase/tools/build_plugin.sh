#!/usr/bin/env bash
# 재렌더 플러그인을 WSL 임시 작업 폴더에 빌드한다(저장소 build/install 미사용).
set -eo pipefail
SRC="$(cd "$(dirname "$0")/render_plugin" && pwd)"
W="$HOME/arena_scratch/feature_showcase/build"
mkdir -p "$W"
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd "$W"
cmake "$SRC" -DCMAKE_BUILD_TYPE=Release > cmake.log 2>&1 || { tail -30 cmake.log; exit 1; }
make 2>&1 | tail -5
ls -la "$W"/*.so
