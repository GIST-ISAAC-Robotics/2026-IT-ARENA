#!/usr/bin/env bash
set +e

repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'

probe_python() {
  label="$1"
  echo "--- ${label} ---"
  printf 'python3_path='; command -v python3
  python3 - <<'PY'
import importlib.util
import sys
print('sys.executable=' + sys.executable)
print('python.version=' + sys.version.replace('\n', ' '))
for name in ('rclpy', 'xacro', 'ackermann_msgs', 'numpy', 'pytest'):
    try:
        spec = importlib.util.find_spec(name)
        print(f'module.{name}=' + (spec.origin or 'namespace') if spec else f'module.{name}=MISSING')
    except Exception as exc:
        print(f'module.{name}=ERROR:{type(exc).__name__}:{exc}')
PY
}

echo '=== WSL focused environment probe ==='
printf 'distro=%s\n' "${WSL_DISTRO_NAME-UNSET}"
echo '=== source ROS Jazzy ==='
source /opt/ros/jazzy/setup.bash
printf 'source_ros_exit=%s\n' "$?"
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO-UNSET}"
printf 'PYTHONPATH=%s\n' "${PYTHONPATH-UNSET}"
probe_python 'after ROS setup'

echo '=== compare /usr/bin/python3 ==='
if [ -x /usr/bin/python3 ]; then
  /usr/bin/python3 - <<'PY'
import importlib.util
import sys
print('sys.executable=' + sys.executable)
print('python.version=' + sys.version.replace('\n', ' '))
for name in ('rclpy', 'xacro', 'ackermann_msgs', 'numpy', 'pytest'):
    try:
        spec = importlib.util.find_spec(name)
        print(f'module.{name}=' + (spec.origin or 'namespace') if spec else f'module.{name}=MISSING')
    except Exception as exc:
        print(f'module.{name}=ERROR:{type(exc).__name__}:{exc}')
PY
else
  echo '/usr/bin/python3=MISSING'
fi

echo '=== dpkg package status ==='
dpkg-query -W -f='${binary:Package} ${Version} ${db:Status-Status}\n' \
  python3-rclpy ros-jazzy-rclpy ros-jazzy-xacro ros-jazzy-ackermann-msgs python3-numpy python3-pytest 2>&1
echo '=== candidate installed package files ==='
for path in /opt/ros/jazzy/lib/python3.12/site-packages/rclpy \
            /opt/ros/jazzy/lib/python3.12/site-packages/xacro \
            /opt/ros/jazzy/lib/python3.12/site-packages/ackermann_msgs \
            /usr/lib/python3/dist-packages/rclpy \
            /usr/lib/python3/dist-packages/xacro \
            /usr/lib/python3/dist-packages/ackermann_msgs; do
  if [ -e "$path" ]; then printf 'PRESENT %s\n' "$path"; else printf 'MISSING %s\n' "$path"; fi
done

echo '=== source workspace local overlay ==='
cd "$repo"
printf 'repo_cd_exit=%s\n' "$?"
source install/local_setup.bash
printf 'source_local_exit=%s\n' "$?"
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO-UNSET}"
printf 'PYTHONPATH=%s\n' "${PYTHONPATH-UNSET}"
probe_python 'after local overlay'

echo '=== add workspace source paths ==='
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
printf 'PYTHONPATH=%s\n' "$PYTHONPATH"
probe_python 'after workspace PYTHONPATH'
