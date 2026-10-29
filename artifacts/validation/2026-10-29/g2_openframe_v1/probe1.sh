#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_openframe_v1
(cd "$base" && /usr/bin/python3 -B -m unittest test_shape -v) > "$base/tests.log" 2>&1 || exit 93
/usr/bin/python3 -B "$base/probe_contacts.py" --output "$PWD/$base/contact_probe1" > "$base/contact_probe1.log" 2>&1
result=$?
echo "$result" > "$base/contact_probe1_exit.txt"
exit "$result"
