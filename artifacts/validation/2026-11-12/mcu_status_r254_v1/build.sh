#!/usr/bin/env bash
# 관측용 상태3필드 후보. 실제 구동 코드 없는 sensor_link 앱만 빌드한다.
set -euo pipefail
root=$(pwd)
a="$root/artifacts/validation/2026-11-12/mcu_status_r254_v1"
stage=/home/jinhyeong/.cache/it-arena/r254-status-link
base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
st=/home/jinhyeong/.cache/it-arena/lsm6dsox-pid_20261111
export IDF_PATH="$base/esp-idf" IDF_TOOLS_PATH="$base/tools" IDF_PYTHON_ENV_PATH="$base/python-v3"
export PATH="$IDF_PYTHON_ENV_PATH/bin:/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin:$PATH"
export IDF_COMPONENT_MANAGER=0 IDF_PY_BUILD_JOBS=4 ST_DRIVER_DIR="$st"
test ! -e "$stage"
test "$(git -C "$st" rev-parse HEAD)" = 2e86793125f7f46b6195fce93a2b3cb76a51e564
git -C "$st" diff --exit-code
mkdir -p "$stage" "$a/build"
cp -a src/arena_vehicle_interface/portable "$stage/portable"
cp -a src/arena_vehicle_interface/firmware_sensor_link "$stage/firmware_sensor_link"
cp "$a/portable/actuation_tx.hpp" "$stage/portable/actuation_tx.hpp"
python - "$stage/firmware_sensor_link/main/main.cpp" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]);s=p.read_text()
before='  aa::TxConfig tx;'
assert s.count(before)==1
p.write_text(s.replace(before,before+'\n  tx.offer_status_v1 = true;'))
PY
tar -czf "$a/build/source_snapshot.tar.gz" -C "$stage" portable firmware_sensor_link
python "$IDF_PATH/tools/idf.py" -C "$stage/firmware_sensor_link" -B "$stage/build" build
python "$IDF_PATH/tools/idf.py" -C "$stage/firmware_sensor_link" -B "$stage/build" size > "$a/build/size.txt"
cp "$stage/build/arena_sensor_link.bin" "$a/build/"
sha256sum "$stage/build/arena_sensor_link.bin" > "$a/build/firmware.sha256"
xtensa-esp32s3-elf-nm -S -C "$stage/build/arena_sensor_link.elf" > "$a/build/symbols.txt"
git -C "$IDF_PATH" rev-parse HEAD > "$a/build/idf_commit.txt"
