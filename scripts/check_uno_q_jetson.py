#!/usr/bin/env python3
"""Jetson의 파일·장치 등록·설정 조회만 한다. UART open, GPIO, 출력 변경 없음."""
import argparse
import glob
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess


def read_text(path):
    try:
        return Path(path).read_text(errors="replace").replace("\x00", "").strip()
    except OSError:
        return None


def query(command):
    if shutil.which(command[0]) is None:
        return {"available": False}
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
        return {"available": True, "returncode": result.returncode,
                "stdout": result.stdout[-12000:], "stderr": result.stderr[-2000:]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"available": True, "error": str(error)}


def inspect(mcu_port=None, servo_port=None, power_status=False):
    paths = sorted(set(glob.glob("/dev/ttyTHS*") + glob.glob("/dev/serial/by-id/*")))
    for name in (mcu_port, servo_port):
        if name and name not in paths:
            paths.append(name)
    ports = []
    for name in paths:
        path = Path(name)
        ports.append({"path": name, "exists": path.exists(), "resolved": str(path.resolve()),
                      "readable": os.access(path, os.R_OK), "writable": os.access(path, os.W_OK),
                      "opened": False})
    report = {"schema": "uno_q_jetson_readonly_preflight_v1", "system": platform.system(),
              "architecture": platform.machine(), "python": platform.python_version(),
              "model": read_text("/proc/device-tree/model"), "l4t_release": read_text("/etc/nv_tegra_release"),
              "kernel_command_line": read_text("/proc/cmdline"), "ports": ports,
              "modules": {name: importlib.util.find_spec(name) is not None for name in ("serial", "rclpy", "sensor_msgs", "std_msgs", "std_srvs", "ackermann_msgs", "numpy", "yaml", "cv2")},
              "ros_distro": os.environ.get("ROS_DISTRO"), "hardware_ports_opened": False,
              "pinmux_changed": False, "outputs_enabled": False,
              "pinmux_verified": False, "mcu_port_selected": mcu_port, "servo_port_selected": servo_port,
              "notes": ["ttyTHS 번호 존재만으로 J12 8/10 대응을 확정하지 않는다.",
                        "이 조회는 배선·전압·MCU 응답·서보 ACK 확인을 대신하지 않는다."]}
    if power_status:
        report["power_status"] = {"nvpmodel": query(["nvpmodel", "-q", "--verbose"]),
                                  "clocks": query(["jetson_clocks", "--show"])}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcu-port")
    parser.add_argument("--servo-port")
    parser.add_argument("--power-status", action="store_true", help="nvpmodel -q / jetson_clocks --show 조회")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = inspect(args.mcu_port, args.servo_port, args.power_status)
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    print(text, end="")


if __name__ == "__main__":
    main()
