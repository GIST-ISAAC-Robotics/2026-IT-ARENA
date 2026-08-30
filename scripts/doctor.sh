#!/usr/bin/env bash
set -u

failures=0

# Make the script independent of the caller's shell state. A fresh WSL shell
# does not put ROS or the vendored Gazebo CLI on PATH until this is sourced.
if [[ -z "${ROS_DISTRO:-}" && -r /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

check_command() {
  local name="$1"
  if command -v "${name}" >/dev/null 2>&1; then
    printf '[ok] %-12s %s\n' "${name}" "$(command -v "${name}")"
  else
    printf '[missing] %s\n' "${name}"
    failures=$((failures + 1))
  fi
}

echo "IT ARENA environment doctor"
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  echo "OS: ${PRETTY_NAME}"
fi
echo "Kernel: $(uname -r)"

check_command ros2
check_command colcon
check_command gz
check_command rosdep
check_command python3

if command -v nvidia-smi >/dev/null 2>&1; then
  gpu_line="$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -n 1)"
  echo "[ok] GPU          ${gpu_line}"
elif [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
  gpu_line="$(/usr/lib/wsl/lib/nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -n 1)"
  echo "[ok] GPU          ${gpu_line}"
else
  echo "[warning] NVIDIA GPU forwarding not detected"
fi

if python3 - <<'PY'
import cv2
assert hasattr(cv2, "aruco")
print(f"[ok] OpenCV       {cv2.__version__}, aruco available")
PY
then
  :
else
  echo "[missing] OpenCV ArUco support"
  failures=$((failures + 1))
fi

if [[ "${failures}" -ne 0 ]]; then
  echo "Doctor found ${failures} missing requirement(s)." >&2
  exit 1
fi

echo "Environment baseline passed."
