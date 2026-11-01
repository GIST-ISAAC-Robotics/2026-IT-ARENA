set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
cd artifacts/validation/2026-11-01/course_compare_v1
python3 -B rejoin_table.py rejoin_table.json 2>&1 | cut -c1-260
python3 -B replay_closing_on_paced.py run_paced closing_on_paced_inputs.json 2>&1 | tail -2 | cut -c1-700
printf 'ACTUAL_EXIT_CODE=%s\n' "$?"
