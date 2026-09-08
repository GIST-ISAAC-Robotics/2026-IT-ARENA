#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../../../.."
source /opt/ros/jazzy/setup.bash
python3 artifacts/validation/2026-09-08/gpu_final/run.py
