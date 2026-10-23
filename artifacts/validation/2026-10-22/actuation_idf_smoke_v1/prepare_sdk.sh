#!/usr/bin/env bash
# 별도 사용자 캐시에서 공식 SDK/Python 빌드 의존성만 준비. 시스템 패키지·장치 연결 없음.
set -euo pipefail
task_base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$task_base/esp-idf"
export IDF_TOOLS_PATH="$task_base/tools"
export IDF_PYTHON_ENV_PATH="$task_base/python"
mkdir -p "$task_base"
if [[ ! -d "$IDF_PATH" ]]; then
  git clone --depth 1 --branch v6.1 --single-branch https://github.com/espressif/esp-idf.git "$IDF_PATH"
fi
test "$(git -C "$IDF_PATH" rev-parse HEAD)" = fff9895c82d744c7237be8847347bdd1b07c6643
git -C "$IDF_PATH" submodule update --init --depth 1 -- \
  components/heap/tlsf \
  components/bootloader/subproject/components/micro-ecc/micro-ecc \
  components/mbedtls/mbedtls
python3 "$IDF_PATH/tools/idf_tools.py" install-python-env --features core
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip install 'ninja==1.13.0'
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip check
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip freeze
git -C "$IDF_PATH" submodule status
du -sh "$task_base"
