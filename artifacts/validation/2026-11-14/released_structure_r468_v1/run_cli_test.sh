#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")/../../../.."
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts:${PYTHONPATH:-}"
python3 -m pytest -q tests/test_guard_curve_trial.py::test_explicit_cli_and_evidence_cannot_claim_release_from_guard > artifacts/validation/2026-11-14/released_structure_r468_v1/integration_pytest_cli.log 2>&1
