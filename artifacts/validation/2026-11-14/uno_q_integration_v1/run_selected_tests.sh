#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest \
  tests/test_uno_q_bundle.py tests/test_uno_q_quick_setup.py \
  tests/test_uno_q_host*.py tests/test_uno_q_firmware.py \
  tests/test_imu_quick_calibration*.py tests/test_quick_self_mask.py \
  -q -p no:cacheprovider \
  --basetemp=/tmp/it-arena-uno-q-integration-20261114 \
  --junitxml=artifacts/validation/2026-11-14/uno_q_integration_v1/pytest.xml
