#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/corner_path_budget_v1
name=${1:?name}
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh /usr/bin/python3 -B "$A/$name.py" > "$A/${name}_v2.log" 2>&1
code=$?
echo "PYTHON_EXIT=$code" > "$A/${name}_v2.exit"
exit "$code"
