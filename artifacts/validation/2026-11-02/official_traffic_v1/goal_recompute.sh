#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
for n in p1 r1 c1; do python3 -B "$A/goal_eval.py" "$A/$n" src/arena_gazebo/worlds/it_arena_official; done > "$A/goal_recompute.log" 2>&1
echo "GOAL=$?"
rm -f "$A/goal_eval_negative.json"
python3 -B "$A/negative.py" > "$A/negative.log" 2>&1
echo "NEGATIVE=$?"
