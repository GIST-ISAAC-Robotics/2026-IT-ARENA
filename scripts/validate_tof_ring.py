#!/usr/bin/env python3
"""실제 Gazebo에서 ToF 저상 표적 응답을 관측합니다. 자율주행 제어기가 아닙니다."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

import cv2
import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[1]


def xyz_points(message):
    fields = {field.name: field.offset for field in message.fields}
    order = ">" if message.is_bigendian else "<"
    result = []
    for row in range(message.height):
        for col in range(message.width):
            start = row * message.row_step + col * message.point_step
            result.append(tuple(struct.unpack_from(order + "f", message.data, start + fields[key])[0]
                                for key in ("x", "y", "z")))
    return result


def target_hits(points, distance, height, sensor_height, width=.15, thickness=.03):
    # 노면을 표적 검출로 세지 않으며 알려진 시험 표적의 상자 부피만 검사합니다.
    return [point for point in points if all(math.isfinite(value) for value in point)
            and distance - .004 <= point[0] <= distance + thickness + .004
            and abs(point[1]) <= width / 2 + .004
            and .004 <= point[2] + sensor_height <= height + .004]


def target_sdf(modules, distance, height):
    links = []
    for module in modules:
        x, y, _ = module["xyz_m"]
        yaw = module["rpy_rad"][2]
        x = -3 + x + (distance + .015) * math.cos(yaw)
        y = -3 + y + (distance + .015) * math.sin(yaw)
        links.append(f'''<link name="{module['name']}">
          <pose>{x} {y} {height / 2} 0 0 {yaw}</pose>
          <visual name="target"><geometry><box><size>.03 .15 {height}</size></box></geometry>
            <material><ambient>.85 .22 .03 1</ambient><diffuse>.9 .3 .06 1</diffuse></material>
          </visual></link>''')
    # 검사 전용 정적 광학 표적이며 물리 충돌이나 차량 제어에 연결하지 않습니다.
    return '<sdf version="1.10"><model name="tof_validation_targets"><static>true</static>' + ''.join(links) + '</model></sdf>'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("low_latency_4x4_60", "tracking_8x8_15"),
                        default="tracking_8x8_15")
    parser.add_argument("--output", type=Path, default=REPO / "artifacts/tests/tof_ring")
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("검사 출력은 프로젝트 artifacts 아래여야 합니다.")
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(70 + os.getpid() % 100)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["GZ_PARTITION"] = f"it_arena_tof_validation_{os.getpid()}"
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, PointCloud2

    config = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text(encoding="utf-8"))
    tof = config["vehicle"]["sensors"]["tof_ring"]
    modules = tof["modules"]
    state = {}
    rclpy.init()
    node = rclpy.create_node("tof_ring_validation")
    subscriptions = []
    for module in modules:
        subscriptions.append(node.create_subscription(
            PointCloud2, module["topic"] + "/points",
            lambda message, name=module["name"]: state.update({name: message}), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Clock, "/clock", lambda m: state.update(clock=m), 10))
    subscriptions.append(node.create_subscription(
        Image, "/sim/tof_inspection/image", lambda m: state.update(image=m), qos_profile_sensor_data))
    inputs = ("src/arena_description/config/vehicle.yaml", "src/arena_description/models/arena_car/model.sdf.xacro",
              "src/arena_bringup/launch/simulation.launch.py", "scripts/validate_tof_ring.py")
    report = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(), "passed": False, "profile": args.profile,
        "scope": "정지 차량의 영역 중심 광선·저상 광학 표적 검사. 전방위 무사각·실물 ToF 성능·회피 검증 아님.",
        "input_sha256": {path: hashlib.sha256((REPO / path).read_bytes()).hexdigest() for path in inputs},
        "scenarios": [],
    }
    log = (output / "simulation.log").open("w", encoding="utf-8")
    processes = []

    def wait_for(predicate, timeout=60):
        deadline = time.monotonic() + timeout
        while not predicate():
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("검사 프로세스가 조기 종료했습니다. simulation.log를 확인하십시오.")
            if time.monotonic() > deadline:
                raise TimeoutError(f"관측 시간 초과: {sorted(state)}")
            rclpy.spin_once(node, timeout_sec=.02)

    def sim_time():
        stamp = state["clock"].clock
        return stamp.sec + stamp.nanosec * 1e-9

    def settle(duration=.4):
        minimum = sim_time() + duration
        wait_for(lambda: sim_time() >= minimum and all(
            state[module["name"]].header.stamp.sec + state[module["name"]].header.stamp.nanosec * 1e-9 >= minimum
            for module in modules))

    def service(name, kind, request):
        result = subprocess.run(["gz", "service", "-s", "/world/it_arena_track/" + name,
                                 "--reqtype", kind, "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                                 "--req", request], capture_output=True, text=True, timeout=10)
        if result.returncode or "data: true" not in result.stdout:
            raise RuntimeError(f"시험 장면 요청 실패: {name}: {result.stdout} {result.stderr}")

    def spawn(sdf):
        service("create", "gz.msgs.EntityFactory", "sdf: " + json.dumps(sdf))

    try:
        processes.append(subprocess.Popen([
            "ros2", "launch", "--noninteractive", "arena_bringup", "simulation.launch.py", "headless:=true",
            "depth_camera:=false", "d435i_profile:=low_load_30", f"tof_profile:={args.profile}"],
            cwd=REPO, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True))
        pose_process = subprocess.Popen([
            "gz", "topic", "-e", "--json-output", "-t", "/world/it_arena_track/dynamic_pose/info"],
            stdout=subprocess.PIPE, stderr=log, text=True, start_new_session=True)
        processes.append(pose_process)

        def poses():
            for line in pose_process.stdout:
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue
                for pose in data.get("pose", []):
                    if pose.get("name") == "arena_car":
                        state["world_pose"] = pose

        threading.Thread(target=poses, daemon=True).start()
        print(f"[{args.profile}] 여섯 ToF와 시험 차량 준비", flush=True)
        wait_for(lambda: "world_pose" in state and "clock" in state and all(m["name"] in state for m in modules))
        service("set_pose", "gz.msgs.Pose", 'name: "arena_car", position: {x: -3, y: -3, z: .01}, orientation: {w: 1}')

        def is_aligned():
            p = state["world_pose"]["position"]
            q = state["world_pose"]["orientation"]
            return abs(p.get("x", 0) + 3) < .002 and abs(p.get("y", 0) + 3) < .002 and abs(p.get("z", 0)) < .002 and all(
                abs(q.get(key, 0)) < .002 for key in ("x", "y", "z"))

        wait_for(is_aligned)
        settle()
        if not is_aligned():
            raise AssertionError("평지 시험 위치·자세가 유지되지 않습니다.")
        report["vehicle_test_pose"] = state["world_pose"]

        for distance, required in ((.18, True), (.50, args.profile == "tracking_8x8_15")):
            spawn(target_sdf(modules, distance, .05))
            settle()
            results = {}
            for module in modules:
                points = xyz_points(state[module["name"]])
                hits = target_hits(points, distance, .05, module["xyz_m"][2])
                results[module["name"]] = {"hit_count": len(hits), "hits_sensor_xyz_m": hits,
                                           "point_count": len(points)}
            detected = all(value["hit_count"] > 0 for value in results.values())
            report["scenarios"].append({"near_face_distance_m": distance, "target_size_m": [.03, .15, .05],
                                        "required": required, "all_six_detected": detected, "modules": results})
            print(f"[{args.profile}] 5 cm 높이 표적·거리 {distance} m: "
                  f"{sum(value['hit_count'] > 0 for value in results.values())}/6 방향 감지", flush=True)
            service("remove", "gz.msgs.Entity", 'name: "tof_validation_targets", type: MODEL')
            settle()
            if required and not detected:
                raise AssertionError(f"필수 저상 표적 검사 미통과: {distance} m")

        camera_sdf = '''<sdf version="1.10"><model name="tof_validation_camera"><static>true</static>
          <pose>-2.65 -3.35 .30 0 .451 2.35619449</pose><link name="camera">
            <sensor name="inspection" type="camera"><always_on>true</always_on><update_rate>2</update_rate>
              <topic>/sim/tof_inspection/image</topic><camera><horizontal_fov>.75</horizontal_fov>
                <image><width>960</width><height>640</height><format>R8G8B8</format></image>
                <clip><near>.01</near><far>5</far></clip>
              </camera></sensor></link></model></sdf>'''
        spawn(camera_sdf)
        processes.append(subprocess.Popen([
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/sim/tof_inspection/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True))
        wait_for(lambda: "image" in state)
        message = state["image"]
        if message.encoding != "rgb8":
            raise AssertionError(f"예상하지 않은 검사 영상 형식: {message.encoding}")
        rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)[
            :, :message.width * 3].reshape(message.height, message.width, 3)
        if not cv2.imwrite(str(output / "car_tof_oblique.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError("검사 카메라 이미지 저장 실패")
        report["screenshot"] = "car_tof_oblique.png"
        # 동적으로 만든 검사 카메라는 실행 중에 먼저 제거합니다. 렌더링 센서의
        # 해제와 서버 종료가 동시에 일어나는 경로를 피하고 프레임 갱신도 확인합니다.
        service("remove", "gz.msgs.Entity", 'name: "tof_validation_camera", type: MODEL')
        settle(.5)
        report["inspection_camera_removed_before_shutdown"] = True
        report["passed"] = True
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    # ROS launch의 자체 SIGINT/SIGTERM 단계(5+10초)가 끝나기 전에
                    # 부모를 동시에 종료시키지 않습니다.
                    process.wait(timeout=25 if process is processes[0] else 10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
        log.close()
        log_text = (output / "simulation.log").read_text(encoding="utf-8", errors="replace")
        errors = [line for line in log_text.splitlines() if "[ERROR]" in line or "process has died" in line or "Traceback" in line]
        report["launch_exit_code"] = processes[0].returncode if processes else None
        report["shutdown_clean"] = bool(processes) and processes[0].returncode == 0 and not errors
        if not report["shutdown_clean"]:
            report["passed"] = False
            report["process_errors"] = errors
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
