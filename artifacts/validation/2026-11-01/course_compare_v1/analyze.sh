set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
cd artifacts/validation/2026-11-01/course_compare_v1
python3 -B compare_course.py comparison.json 2>&1 | cut -c1-420
echo == candidate bypass reasons
python3 -B stall_trace.py run_paced 4 40 2>&1 | tail -60 | cut -c1-190
