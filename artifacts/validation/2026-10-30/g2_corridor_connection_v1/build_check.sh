#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_corridor_connection_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B -m pytest -q -rs -p no:cacheprovider tests/test_g2_corridor_provenance.py tests/test_g2_handoff_provenance.py > "$base/provenance.log" 2>&1
echo "provenance $?"
colcon build --packages-select arena_autonomy --symlink-install > "$base/build.log" 2>&1
echo "build $?"
set +u
source install/local_setup.bash
unset PYTHONPATH
cd /tmp
/usr/bin/python3 -B - <<'P' > "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/$base/installed.log" 2>&1
import hashlib, inspect
import arena_autonomy.bypass_corridor_trial as corridor
import arena_autonomy.bypass_entry_follow as follow
import arena_autonomy.g2_entry_node as node
print('corridor', corridor.__file__)
print('policies', follow.CORRIDOR_POLICIES, follow.BypassExecutionConfig().corridor_policy)
print('node_param', "'g2_corridor_policy','off'" in inspect.getsource(node))
for module in (corridor, follow, node):
    print(module.__name__.split('.')[-1], hashlib.sha256(open(module.__file__, 'rb').read()).hexdigest()[:16])
P
cat "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/$base/installed.log"
ros2 pkg executables arena_autonomy | grep g2_entry_trial
