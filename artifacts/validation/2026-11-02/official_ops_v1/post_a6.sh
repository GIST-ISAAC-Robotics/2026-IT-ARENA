#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -I -B "$A/goal_eval.py" "$A/a6"; echo "GOAL_EXIT=$?"
REQ=$(python3 -c "import json;d=json.load(open('$A/a6/report.json'));print([s['request'] for s in d['steps'] if s['step']=='press_stop'][0])")
python3 -B scripts/race_event_export.py --log "$A/a6/ops_log.jsonl" --request "$REQ" --decisions "$A/a6/follow_decisions.jsonl" --truth "$A/a6/ego_poses.jsonl" --before-s 8 --after-s 8 --output "$A/a6_first_stop_timeline"; echo "EXPORT_EXIT=$?"
python3 -B -m pytest -q -p no:cacheprovider --tb=line tests/test_source_provenance_r0_plus_b3.py 2>&1 | tail -8
