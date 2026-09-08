#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../../../.."
source /opt/ros/jazzy/setup.bash
cmake -S artifacts/validation/2026-09-08/gpu_final -B build/gpu_final -DCMAKE_BUILD_TYPE=Release
cmake --build build/gpu_final -j2
