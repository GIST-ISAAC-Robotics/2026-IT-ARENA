#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$A/prepare_contract.py"
python3 -B -m pytest -q -p no:cacheprovider --tb=line tests/test_source_provenance_r0_plus_b3.py 2>&1 | tail -6
python3 -B -m pytest -q -p no:cacheprovider tests --tb=line -x -q --co -q 2>&1 | tail -1
timeout 480 python3 -B -m pytest -q -p no:cacheprovider tests --tb=line > "$A/tests_full.log" 2>&1
echo "FULL=$?"; tail -18 "$A/tests_full.log" | cut -c1-200
