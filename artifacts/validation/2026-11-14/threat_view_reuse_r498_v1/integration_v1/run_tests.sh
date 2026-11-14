#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")/../../../../.."
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:${PYTHONPATH:-}"
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_threat_view_transfer.py tests/test_traffic_solo_gap.py tests/test_pass_release_trial.py > artifacts/validation/2026-11-14/threat_view_reuse_r498_v1/integration_v1/focused_pytest.log 2>&1
