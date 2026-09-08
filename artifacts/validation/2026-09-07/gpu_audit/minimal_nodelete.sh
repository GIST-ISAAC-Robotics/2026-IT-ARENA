#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
audit_dir="$(pwd)/artifacts/validation/2026-09-07/gpu_audit"
g++ -shared -fPIC -Wall -Wextra -o "$audit_dir/libd3d12_nodelete_trial.so" "$audit_dir/d3d12_nodelete_trial.cpp" -ldl
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GZ_PARTITION=arena_gpu_minimal_nodelete
export LD_LIBRARY_PATH="$audit_dir:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD=libd3d12_nodelete_trial.so
gz sim -r -s --iterations 100 sensors_demo.sdf
