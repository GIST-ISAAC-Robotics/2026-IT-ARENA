set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
B=artifacts/validation/2026-11-01/follow_continuity_v1
python3 -B -m pytest -q -p no:cacheprovider tests/test_low_motion_reserve.py tests/test_follow_closing_reserve.py tests/test_recent_front_motion.py tests/test_g1_handoff_eval.py tests/test_bypass_handoff.py tests/test_source_provenance_r0_plus_b3.py > "$B/pytest_focus_v2.log" 2>&1
printf 'ACTUAL_EXIT_CODE=%s\n' "$?" >> "$B/pytest_focus_v2.log"
tail -3 "$B/pytest_focus_v2.log" | cut -c1-140
for c in curve stop fast; do python3 -B "$B/run_paced.py" "$c" --prepare-only || exit 1; done
echo PREPARED
