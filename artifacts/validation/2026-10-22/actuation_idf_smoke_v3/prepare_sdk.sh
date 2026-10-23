#!/usr/bin/env bash
# Ubuntu ensurepip 누락을 시스템 패키지 변경 없이 전용 venv 안에서 해결한다.
set -euo pipefail
task_root='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
evidence="$task_root/artifacts/validation/2026-10-22/actuation_idf_smoke_v3"
task_base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$task_base/esp-idf"
export IDF_TOOLS_PATH="$task_base/tools"
export IDF_PYTHON_ENV_PATH="$task_base/python-v3"
test ! -e "$IDF_PYTHON_ENV_PATH"
python3 -m venv --without-pip "$IDF_PYTHON_ENV_PATH"
curl --fail --location --retry 2 --max-time 120 https://bootstrap.pypa.io/get-pip.py -o "$task_base/get-pip.py"
sha256sum "$task_base/get-pip.py" > "$evidence/pip_bootstrap.sha256"
"$IDF_PYTHON_ENV_PATH/bin/python" "$task_base/get-pip.py"
python3 "$IDF_PATH/tools/idf_tools.py" install-python-env --features core
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip install 'ninja==1.13.0'
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip check
"$IDF_PYTHON_ENV_PATH/bin/python" -m pip freeze > "$evidence/python_packages.txt"
du -sh "$task_base"
