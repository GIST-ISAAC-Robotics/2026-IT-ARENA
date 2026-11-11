#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=199
cd /home/q/it_arena_current_rebase_r128_20261111
/usr/bin/python3 -B probe_r137.py --root . --output r137_rebase --mode rebase
tar -czf r137_results.tar.gz r137_rebase
sha256sum r137_results.tar.gz
