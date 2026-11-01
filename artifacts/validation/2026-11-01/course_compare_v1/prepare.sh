set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
B=artifacts/validation/2026-11-01/course_compare_v1
python3 -B "$B/compare_course.py" "$B/base_check.json" --check-base-only 2>&1 | cut -c1-300
python3 -B "$B/run_course.py" --prepare-only 2>&1 | tail -3
