#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as the normal WSL user, not root." >&2
  exit 1
fi

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed. Run scripts/setup_wsl.sh as root first." >&2
  exit 1
fi

rosdep update

marker_start="# >>> 2026-it-arena >>>"
marker_end="# <<< 2026-it-arena <<<"

if ! grep -Fq "${marker_start}" "${HOME}/.bashrc"; then
  {
    echo
    echo "${marker_start}"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    echo "${marker_end}"
  } >> "${HOME}/.bashrc"
fi

echo "User environment configured. Open a new WSL shell or source /opt/ros/jazzy/setup.bash."
