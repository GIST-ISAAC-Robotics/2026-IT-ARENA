#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=199
cd /home/q/it_arena_current_rebase_r128_20261111
/usr/bin/python3 -B probe_r138_v2.py --root . --output r138_rebase_v2 --mode rebase
tar -czf r138_results_v2.tar.gz r138_rebase_v2
sha256sum r138_results_v2.tar.gz
