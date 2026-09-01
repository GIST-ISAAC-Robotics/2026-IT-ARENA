#!/usr/bin/env python3
"""공식 런타임 코스를 여러 고정 카메라 시점에서 PNG로 기록합니다."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np


REPO = Path(__file__).resolve().parents[1]
WORLD_NAME = "it_arena_track"
WIDTH = 2300
HEIGHT = 1500
HORIZONTAL_FOV_RAD = 1.2
TRACK_CENTER = (5.606305, 7.249605, 0.1)

# Gazebo 카메라는 로컬 +X 방향을 바라본다. 각 자세는 아래 위치에서
# TRACK_CENTER를 바라보도록 계산하며, 코스 전체와 약간의 여백이 함께 들어온다.
VIEWS = (
    {
        "name": "track_top",
        "label": "정수직 전체",
        "position": (5.606305, 7.249605, 14.0),
    },
    {
        "name": "track_south_oblique",
        "label": "남측 사선 전체",
        "position": (5.606305, -7.0, 12.0),
    },
    {
        "name": "track_west_oblique",
        "label": "서측 사선 전체",
        "position": (-5.0, 7.249605, 12.0),
    },
    {
        "name": "track_east_oblique",
        "label": "동측 사선 전체",
        "position": (16.2, 7.249605, 12.0),
    },
    {
        "name": "track_north_oblique",
        "label": "북측 사선 전체",
        "position": (5.606305, 21.0, 12.0),
    },
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position: tuple[float, float, float]) -> tuple[float, float, float, float]:
    dx = TRACK_CENTER[0] - position[0]
    dy = TRACK_CENTER[1] - position[1]
    dz = TRACK_CENTER[2] - position[2]
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if horizontal > 1e-12 else 0.0
    pitch = math.atan2(-dz, horizontal)
    sp, cp = math.sin(pitch / 2), math.cos(pitch / 2)
    sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


def camera_sdf(view: dict[str, object], topic: str) -> str:
    px, py, pz = view["position"]
    qx, qy, qz, qw = look_at_quaternion(view["position"])
    name = view["name"]
    return f'''<sdf version="1.10">
      <model name="capture_{name}">
        <static>true</static>
        <pose rotation_format="quat_xyzw">{px} {py} {pz} {qx} {qy} {qz} {qw}</pose>
        <link name="camera_link">
          <sensor name="camera" type="camera">
            <always_on>true</always_on>
            <update_rate>2</update_rate>
            <topic>{topic}</topic>
            <camera>
              <horizontal_fov>{HORIZONTAL_FOV_RAD}</horizontal_fov>
              <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
              <clip><near>0.1</near><far>100</far></clip>
            </camera>
          </sensor>
        </link>
      </model>
    </sdf>'''


def gazebo_service(name: str, kind: str, request: str) -> None:
    result = subprocess.run(
        [
            "gz", "service", "-s", f"/world/{WORLD_NAME}/{name}",
            "--reqtype", kind, "--reptype", "gz.msgs.Boolean",
            "--timeout", "10000", "--req", request,
        ],
        capture_output=True,
        text=True,
        timeout=15,
    )
    if result.returncode or "data: true" not in result.stdout:
        raise RuntimeError(f"Gazebo {name} 요청 실패: {result.stdout} {result.stderr}")


def stop_process(process: subprocess.Popen, timeout: float = 12.0) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def image_array(message) -> np.ndarray:
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    if message.encoding == "rgb8":
        rgb = rows[:, :message.width * 3].reshape(message.height, message.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if message.encoding == "bgr8":
        return rows[:, :message.width * 3].reshape(message.height, message.width, 3)
    if message.encoding == "rgba8":
        rgba = rows[:, :message.width * 4].reshape(message.height, message.width, 4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    if message.encoding == "bgra8":
        bgra = rows[:, :message.width * 4].reshape(message.height, message.width, 4)
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    raise AssertionError(f"예상하지 않은 카메라 인코딩: {message.encoding}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO / "artifacts/screenshots/2026-09-02/official_update",
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("캡처 출력은 프로젝트 artifacts 아래여야 합니다.")
    output.mkdir(parents=True, exist_ok=True)

    os.environ["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 50)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["GZ_PARTITION"] = f"it_arena_track_capture_{os.getpid()}"
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    world = REPO / "src/arena_gazebo/worlds/it_arena_official/world.sdf"
    vehicle = REPO / "src/arena_description/config/vehicle.yaml"
    report = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "passed": False,
        "scope": "v2026.09.02 기반 팀 런타임 코스의 고정 외부 카메라 기록",
        "world_sha256": sha256(world),
        "vehicle_config_sha256": sha256(vehicle),
        "resolution_px": [WIDTH, HEIGHT],
        "views": [],
    }
    log_path = output / "capture.log"
    log = log_path.open("w", encoding="utf-8")
    processes: list[subprocess.Popen] = []
    spawned: list[str] = []
    state: dict[str, object] = {}
    rclpy.init()
    node = rclpy.create_node("official_track_view_capture")
    clock_subscription = node.create_subscription(
        Clock, "/clock", lambda message: state.update(clock=message), 10
    )

    def wait_for(predicate, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while not predicate():
            if processes and processes[0].poll() is not None:
                raise RuntimeError("Gazebo 서버가 캡처 전에 종료했습니다. capture.log를 확인하십시오.")
            if time.monotonic() > deadline:
                raise TimeoutError(f"캡처 관측 시간 초과: {sorted(state)}")
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        simulation = subprocess.Popen(
            [
                "ros2", "launch", "--noninteractive", "arena_bringup", "simulation.launch.py",
                "headless:=true", "track:=official", "render_sensors:=false",
                "tof_safety:=false", "depth_camera:=false",
            ],
            cwd=REPO,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        processes.append(simulation)
        wait_for(lambda: "clock" in state, args.timeout)

        for view in VIEWS:
            name = str(view["name"])
            model_name = f"capture_{name}"
            topic = f"/sim/official_track_capture/{name}/image"
            frame_state = {"count": 0, "message": None}

            def receive(message, target=frame_state):
                target["count"] += 1
                target["message"] = message

            subscription = node.create_subscription(Image, topic, receive, qos_profile_sensor_data)
            gazebo_service("create", "gz.msgs.EntityFactory", "sdf: " + json.dumps(camera_sdf(view, topic)))
            spawned.append(model_name)
            bridge = subprocess.Popen(
                [
                    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                    f"{topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
            processes.append(bridge)
            # 두 번째 프레임을 사용하여 모델·재질이 첫 렌더에 모두 반영되도록 한다.
            wait_for(lambda: frame_state["count"] >= 2, args.timeout)
            message = frame_state["message"]
            image = image_array(message)
            if image.shape[:2] != (HEIGHT, WIDTH):
                raise AssertionError(f"{name} 해상도 불일치: {image.shape}")
            grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if float(grayscale.std()) < 5.0:
                raise AssertionError(f"{name} 영상이 단색에 가깝습니다: 표준편차 {grayscale.std():.3f}")
            path = output / f"{name}.png"
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"PNG 저장 실패: {path}")
            report["views"].append(
                {
                    "name": name,
                    "label": view["label"],
                    "file": path.name,
                    "position_m": view["position"],
                    "quaternion_xyzw": look_at_quaternion(view["position"]),
                    "encoding": message.encoding,
                    "grayscale_stddev": float(grayscale.std()),
                    "sha256": sha256(path),
                }
            )
            print(f"[{len(report['views'])}/{len(VIEWS)}] {view['label']}: {path.name}", flush=True)

            gazebo_service("remove", "gz.msgs.Entity", f'name: "{model_name}", type: MODEL')
            spawned.remove(model_name)
            node.destroy_subscription(subscription)
            stop_process(bridge)
            processes.remove(bridge)

        report["passed"] = len(report["views"]) == len(VIEWS)
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        for model_name in reversed(spawned):
            try:
                gazebo_service("remove", "gz.msgs.Entity", f'name: "{model_name}", type: MODEL')
            except Exception as cleanup_error:
                report.setdefault("cleanup_errors", []).append(str(cleanup_error))
        node.destroy_subscription(clock_subscription)
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            stop_process(process, timeout=25.0 if process is processes[0] else 12.0)
        log.close()
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        errors = [
            line for line in log_text.splitlines()
            if "[ERROR]" in line or "process has died" in line or "Traceback" in line
        ]
        report["launch_exit_code"] = processes[0].returncode if processes else None
        report["shutdown_clean"] = bool(processes) and processes[0].returncode == 0 and not errors
        if not report["shutdown_clean"]:
            report["passed"] = False
            report["process_errors"] = errors
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        (output / "capture_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )

    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
