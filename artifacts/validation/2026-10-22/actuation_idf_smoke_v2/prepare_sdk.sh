#!/usr/bin/env bash
# v1은 지정 Python 환경 폴더가 없어 설치기 사전검사에서 실패. 소스/로그는 보존한다.
set -euo pipefail
task_base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$task_base/esp-idf"
export IDF_TOOLS_PATH="$task_base/tools"
export IDF_PYTHON_ENV_PATH="$task_base/python"
test "$(git -C "$IDF_PATH" rev-parse HEAD)" = fff9895c82d744c7237be8847347bdd1b07c6643
mkdir -p "$IDF_PYTHON_ENV_PATH"
python3 "$IDF_PATH/tools/idf_tools.py" install-python-env --features core
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip install 'ninja==1.13.0'
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip check
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip freeze
du -sh "$task_base"
