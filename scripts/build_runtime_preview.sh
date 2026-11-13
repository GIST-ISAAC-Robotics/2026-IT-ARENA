#!/usr/bin/env bash
# 확정 GPIO 기반 preview/명시 drive 후보 빌드. flash는 수행하지 않는다.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
stage=${1:?fresh no-spaces build path required}
st_source=${2:?pinned ST source required}
evidence=${3:?fresh evidence path required}
app=${4:-firmware_runtime_preview}
profile=''
if [[ $# -gt 4 ]]; then
  if [[ $# -ne 6 || $app != firmware_runtime_drive || $5 != --installed-profile ]]; then
    printf 'installed profile is an explicit drive-only option\n' >&2; exit 2
  fi
  profile=$(realpath -e -- "$6")
  python3 "$task_root/scripts/runtime_installed_profile.py" verify --profile-dir "$profile"
fi
case "$app" in
  firmware_runtime_preview) binary=arena_runtime_preview ;;
  firmware_runtime_drive) binary=arena_runtime_drive ;;
  *) printf 'unsupported app: %s\n' "$app" >&2; exit 2 ;;
esac
base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$base/esp-idf" IDF_TOOLS_PATH="$base/tools" IDF_PYTHON_ENV_PATH="$base/python-v3"
export PATH="$IDF_PYTHON_ENV_PATH/bin:/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin:$PATH"
export IDF_COMPONENT_MANAGER=0 IDF_PY_BUILD_JOBS=1 ST_DRIVER_DIR="$st_source"
test ! -e "$stage"
test ! -e "$evidence/build_exit.txt"
test "$(git -C "$st_source" rev-parse HEAD)" = 2e86793125f7f46b6195fce93a2b3cb76a51e564
git -C "$st_source" diff --exit-code
mkdir -p "$stage" "$evidence"
cd "$task_root"
sources=(src/arena_vehicle_interface/portable/*.hpp src/arena_vehicle_interface/firmware_runtime_preview/CMakeLists.txt
  src/arena_vehicle_interface/firmware_runtime_preview/sdkconfig.defaults src/arena_vehicle_interface/firmware_runtime_preview/main/*)
if [[ "$app" = firmware_runtime_drive ]]; then
  sources+=(src/arena_vehicle_interface/firmware_runtime_drive/CMakeLists.txt
    src/arena_vehicle_interface/firmware_runtime_drive/sdkconfig.defaults src/arena_vehicle_interface/firmware_runtime_drive/main/*)
fi
if [[ -n "$profile" ]]; then
  sources+=(scripts/runtime_installed_profile.py
    src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py
    src/arena_vehicle_interface/arena_vehicle_interface/steering_cad_table.py)
  sha256sum "$profile/manifest.json" "$profile/runtime_board.json" "$profile/steering_zero.json" \
    "$profile/mounted_steering_zero.hpp" "$profile/runtime_installed_profile.hpp" > "$evidence/profile_before.sha256"
fi
sha256sum "${sources[@]}" > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot.tar.gz" "${sources[@]}"
finish() {
  result=$?
  sha256sum "${sources[@]}" > "$evidence/sources_after.sha256"
  if ! diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_diff.txt"; then result=1; fi
  if [[ -n "$profile" ]]; then
    if ! sha256sum --check "$evidence/profile_before.sha256" > "$evidence/profile_after_check.txt"; then result=1; fi
  fi
  printf '%s\n' "$result" > "$evidence/build_exit.txt"
  exit "$result"
}
trap finish EXIT
cp -a src/arena_vehicle_interface/portable "$stage/portable"
cp -a src/arena_vehicle_interface/firmware_runtime_preview "$stage/firmware_runtime_preview"
if [[ "$app" = firmware_runtime_drive ]]; then
  cp -a src/arena_vehicle_interface/firmware_runtime_drive "$stage/firmware_runtime_drive"
fi
profile_flags=()
if [[ -n "$profile" ]]; then
  mkdir "$stage/runtime_installed_profile"
  cp "$profile/manifest.json" "$profile/runtime_board.json" "$profile/steering_zero.json" \
    "$profile/mounted_steering_zero.hpp" "$profile/runtime_installed_profile.hpp" "$stage/runtime_installed_profile/"
  python "$task_root/scripts/runtime_installed_profile.py" verify --profile-dir "$stage/runtime_installed_profile" \
    > "$evidence/profile_verify.json"
  cp -a "$stage/runtime_installed_profile" "$evidence/installed_profile"
  # 기존 C++ valid를 그대로 사용한다. native 검사도 장치/ESP-IDF main을 실행하지 않는다.
  g++ -std=c++17 -DARENA_RUNTIME_INSTALLED_PROFILE=1 -I"$stage/portable" \
    -I"$stage/firmware_runtime_drive/main" -I"$stage/runtime_installed_profile" \
    -x c++ - -o "$stage/profile_check" <<'CPP'
#include "board_profile.hpp"
int main() {
  const arena_steering::CadMapping map(arena_steering::kCadSamples,
    sizeof(arena_steering::kCadSamples)/sizeof(arena_steering::kCadSamples[0]));
  return installed_board_profile().valid(map) ? 0 : 2;
}
CPP
  "$stage/profile_check"
  printf 'existing_RuntimeBoardProfile_valid=true; hardware_executed=false\n' > "$evidence/profile_cpp_valid.txt"
  profile_flags=(-D ARENA_RUNTIME_INSTALLED_PROFILE=ON)
fi
python "$IDF_PATH/tools/idf.py" -C "$stage/$app" -B "$stage/build" "${profile_flags[@]}" build
python "$IDF_PATH/tools/idf.py" -C "$stage/$app" -B "$stage/build" size > "$evidence/size.txt"
cp "$stage/build/$binary.bin" "$evidence/"
sha256sum "$stage/build/$binary.bin" > "$evidence/firmware.sha256"
git -C "$IDF_PATH" rev-parse HEAD > "$evidence/idf_commit.txt"
xtensa-esp32s3-elf-nm -S -C "$stage/build/$binary.elf" > "$evidence/symbols.txt"
