#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GZ_PARTITION=arena_gpu_minimal_debug
gdb -batch -nx -ex 'set debuginfod enabled off' -ex 'handle SIGINT nostop noprint pass' -ex run -ex 'thread apply all bt 16' --args /usr/bin/ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -r -s --iterations 100 sensors_demo.sdf
