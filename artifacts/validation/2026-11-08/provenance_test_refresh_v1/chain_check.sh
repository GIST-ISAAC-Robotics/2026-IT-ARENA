cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/tests"
python3 -B artifacts/validation/2026-11-08/provenance_test_refresh_v1/chain_check.py
