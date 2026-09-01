#!/usr/bin/env python3
"""실제 차량 RGB 영상으로 시설 가시성을 검사합니다. 대회용 인지 노드가 아닙니다."""

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
import threading
import time

import cv2
import numpy as np


REPO = Path(__file__).resolve().parents[1]


def yaw_of(q):
    x, y, z, w = (float(q.get(key, default)) for key, default in (("x", 0), ("y", 0), ("z", 0), ("w", 1)))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def image_array(message):
    if message.encoding != "rgb8":
        raise ValueError(f"예상하지 않은 RGB 표현: {message.encoding}")
    return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)[
        :, :message.width * 3].reshape(message.height, message.width, 3).copy()


def detect_markers(rgb):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    return {int(marker_id): corner.reshape(4, 2).tolist()
            for marker_id, corner in zip([] if ids is None else ids.flatten(), corners)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--capture-only", action="store_true", help="현재 출발 위치의 원본 RGB만 저장")
    mode.add_argument("--bump-only", action="store_true", help="방지턱 물리·표시 진단만 실행")
    mode.add_argument("--flat-control", action="store_true", help="방지턱 이전 평지에서 동일 속도의 정지 대조 시험")
    parser.add_argument("--track", choices=("official", "original", "experimental"), default="experimental",
                        help="검사할 트랙 프로필(기존 동작 호환을 위해 기본값은 experimental)")
    parser.add_argument("--case-file", type=Path,
                        help="scene.json의 기본 camera_cases 대신 사용할 JSON 배열(감사용)")
    parser.add_argument("--output", type=Path, default=REPO / "artifacts/tests/facility_visibility")
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("검사 출력은 이 프로젝트 artifacts 아래에만 저장합니다.")
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(70 + os.getpid() % 100)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["GZ_PARTITION"] = f"it_arena_visibility_{os.getpid()}"
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("arena_facility_inspection")
    state = {}
    pose_history = []
    record_poses = False

    def observe(message, key):
        state[key] = message

    subscriptions = [
        node.create_subscription(Image, "/camera/color/image_raw", lambda m: observe(m, "rgb"), QoSProfile(depth=5)),
        node.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: observe(m, "info"), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", lambda m: observe(m, "odom"), 10),
    ]
    drive_publisher = node.create_publisher(AckermannDriveStamped, "/drive", 10)
    world_directories = {"official": "it_arena_official", "original": "it_arena_track",
                         "experimental": "it_arena_experimental"}
    world_directory = REPO / "src/arena_gazebo/worlds" / world_directories[args.track]
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "passed": False,
              "source": "arena_car D435i RGB /camera/color/image_raw", "profile": "low_load_30",
              "track": args.track,
              "capture_only": args.capture_only, "bump_only": args.bump_only, "flat_control": args.flat_control,
              "scope": "정지한 차량의 실제 렌더링 영상. 정답 위치는 시험 배치·검사에만 사용.", "cases": []}
    report["input_sha256"] = {
        str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest() for path in (
            world_directory / "world.sdf", world_directory / "scene.json",
            world_directory / "provenance.json", REPO / "src/arena_description/config/vehicle.yaml")}
    log_path = output / "simulation.log"
    log = log_path.open("w", encoding="utf-8")
    processes = []
    try:
        processes.append(subprocess.Popen([
            "ros2", "launch", "arena_bringup", "simulation.launch.py", "headless:=true",
            f"track:={args.track}", "d435i_profile:=low_load_30"],
            cwd=REPO, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        if not args.capture_only:
            # SceneBroadcaster의 pose.name은 TF 변환 시 보존되지 않습니다.
            # 이름을 임의의 배열 순서로 대체하지 않고 원래 GZ 메시지를 읽습니다.
            pose_process = subprocess.Popen([
                "gz", "topic", "-e", "-t", "/world/it_arena_track/dynamic_pose/info", "--json-output"],
                cwd=REPO, stdout=subprocess.PIPE, stderr=log, text=True, start_new_session=True)
            processes.append(pose_process)

            def observe_poses():
                for line in pose_process.stdout:
                    try:
                        message = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    for entry in message.get("pose", []):
                        if entry.get("name") == "arena_car":
                            stamp = message.get("header", {}).get("stamp", {})
                            entry["sim_stamp_s"] = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9
                            state["world_pose"] = entry
                            if record_poses:
                                pose_history.append(entry)

            threading.Thread(target=observe_poses, daemon=True).start()

        def wait_for(predicate, timeout=60):
            deadline = time.monotonic() + timeout
            next_log_check = 0
            while not predicate():
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError(f"검사 중 프로세스 조기 종료: {log_path}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"시설 관측 시간 초과: {sorted(state)}")
                if time.monotonic() >= next_log_check:
                    errors = [line for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
                              if "process has died" in line or "Segmentation fault" in line or "Failed to load a world" in line]
                    if errors:
                        raise RuntimeError(errors[-1])
                    next_log_check = time.monotonic() + 1
                rclpy.spin_once(node, timeout_sec=.02)

        def sim_time():
            stamp = state["odom"].header.stamp
            return stamp.sec + stamp.nanosec / 1e9

        def settle(duration=.6):
            started = sim_time()
            wait_for(lambda: sim_time() >= started + duration)
            stamp = state["rgb"].header.stamp
            minimum = started + duration
            wait_for(lambda: state["rgb"].header.stamp.sec + state["rgb"].header.stamp.nanosec / 1e9 >= minimum)

        def save_frame(name):
            rgb = image_array(state["rgb"])
            file = output / f"{name}.png"
            if not cv2.imwrite(str(file), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError(f"영상 저장 실패: {file}")
            return rgb, file

        def move_to(x, y, yaw):
            request = (f'name: "arena_car", position: {{x: {x:.9f}, y: {y:.9f}, z: 0.01}}, '
                       f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.12f}, w: {math.cos(yaw / 2):.12f}}}')
            # 서비스 응답만으로 성공을 판단하지 않고 별도 세계 좌표 관측을 대조합니다.
            command = subprocess.Popen([
                "gz", "service", "-s", "/world/it_arena_track/set_pose", "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                wait_for(lambda: command.poll() is not None, timeout=12)
                response = command.communicate()[0]
                if command.returncode or "data: true" not in response:
                    raise RuntimeError(f"시험 배치 요청 실패: {response}")
            finally:
                if command.poll() is None:
                    command.terminate()
                    command.wait(timeout=5)

            def aligned():
                if "world_pose" not in state:
                    return False
                pose = state["world_pose"]
                error = math.atan2(math.sin(yaw_of(pose["orientation"]) - yaw), math.cos(yaw_of(pose["orientation"]) - yaw))
                return math.hypot(pose["position"].get("x", 0) - x, pose["position"].get("y", 0) - y) < .01 and abs(error) < .015

            wait_for(aligned, timeout=20)
            settle()
            if not aligned():
                raise AssertionError("시험 배치 후 차량 위치·방향이 안정적으로 유지되지 않습니다.")

        print("실제 차량 RGB와 위치 관측 준비", flush=True)
        required = ("rgb", "info", "odom") if args.capture_only else ("rgb", "info", "odom", "world_pose")
        wait_for(lambda: all(key in state for key in required))
        settle()
        rgb, file = save_frame("initial_vehicle_view")
        report["initial_frame"] = str(file.relative_to(REPO))
        report["initial_rgb_range"] = [int(rgb.min()), int(rgb.max())]
        report["initial_detected_ids"] = sorted(detect_markers(rgb))
        if not args.capture_only:
            scene = json.loads((world_directory / "scene.json").read_text(encoding="utf-8"))
            camera_cases = scene["facility_inspection"]["camera_cases"]
            if args.case_file:
                case_file = args.case_file.resolve()
                if not case_file.is_relative_to(REPO):
                    raise ValueError("사용자 지정 검사 사례는 프로젝트 폴더 안에 있어야 합니다.")
                camera_cases = json.loads(case_file.read_text(encoding="utf-8"))
                report["case_file"] = str(case_file.relative_to(REPO))
            for case in ([] if args.bump_only or args.flat_control else camera_cases):
                move_to(case["x"], case["y"], case["yaw_rad"])
                rgb, file = save_frame(case["name"])
                detections = detect_markers(rgb)
                result = {**case, "image": str(file.relative_to(REPO)), "detected_ids": sorted(detections),
                          "detected_corners_px": detections, "observed_world_pose": state["world_pose"]}
                if "expected_marker_id" in case:
                    result["passed"] = case["expected_marker_id"] in detections
                else:
                    red = (rgb[:, :, 0] > 160) & (rgb[:, :, 1] < 100) & (rgb[:, :, 2] < 100)
                    count, _, stats, centers = cv2.connectedComponentsWithStats(red.astype(np.uint8), 8)
                    blobs = [{"pixels": int(stats[i, cv2.CC_STAT_AREA]), "center": centers[i].tolist()}
                             for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 10]
                    result["red_blobs"] = blobs
                    # 엉뚱한 빨간 픽셀을 신호등으로 인정하지 않도록, 별도 세계 위치와
                    # 실제 CameraInfo로 투영한 빨간 렌즈 근처에서만 가시성을 판정합니다.
                    import yaml
                    camera = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text(encoding="utf-8"))["vehicle"]["sensors"]["d435i"]
                    vehicle_pose = state["world_pose"]
                    heading = yaw_of(vehicle_pose["orientation"])
                    c, s = math.cos(heading), math.sin(heading)
                    xyz = camera["xyz_m"]
                    cp = [vehicle_pose["position"]["x"] + c * xyz[0] - s * xyz[1],
                          vehicle_pose["position"]["y"] + s * xyz[0] + c * xyz[1],
                          vehicle_pose["position"].get("z", 0) + xyz[2]]
                    lamp = scene["traffic_light"]["lamp_poses"]["red"]
                    dx, dy, dz = [lamp[key] - cp[i] for i, key in enumerate(("x", "y", "z"))]
                    forward, left = c * dx + s * dy, -s * dx + c * dy
                    info = state["info"]
                    expected = [info.k[2] - info.k[0] * left / forward, info.k[5] - info.k[4] * dz / forward]
                    result["expected_red_center_px"] = expected
                    result["passed"] = any(math.dist(blob["center"], expected) < 10 for blob in blobs)
                report["cases"].append(result)
                print(f"{case['name']}: {'PASS' if result['passed'] else 'FAIL'}; ArUco={sorted(detections)}", flush=True)

            bump = scene["speed_bumps"]["bumps"][0]
            heading = bump["yaw_rad"]
            c, s = math.cos(heading), math.sin(heading)
            trial_name = "flat_control" if args.flat_control else "bump_traversal"
            distance = 1.85 if args.flat_control else .65
            move_to(bump["x"] - distance * c, bump["y"] - distance * s, heading)
            bump_rgb, file = save_frame("flat_approach" if args.flat_control else "bump_approach")
            yellow_pixels = int(np.count_nonzero((bump_rgb[:, :, 0] > 140) & (bump_rgb[:, :, 1] > 80)
                                                & (bump_rgb[:, :, 2] < 80)))
            initial = state["world_pose"]["position"].copy()
            initial_z = initial.get("z", 0)
            record_poses = True
            started, deadline, next_publish = sim_time(), time.monotonic() + 50, 0.0
            while sim_time() - started < 7.0:
                now = time.monotonic()
                if now > deadline:
                    raise TimeoutError("방지턱 통과 중 시뮬레이션 진행 시간 초과")
                if now >= next_publish:
                    command = AckermannDriveStamped()
                    command.drive.speed = .16
                    drive_publisher.publish(command)
                    next_publish = now + .05
                rclpy.spin_once(node, timeout_sec=.01)
            drive_publisher.publish(AckermannDriveStamped())
            stopped_at = sim_time()
            stable_since = None
            stop_stable = False
            speed_trace = []
            deadline = time.monotonic() + 60
            while sim_time() - stopped_at < 6.0:
                if time.monotonic() > deadline:
                    raise TimeoutError("정지 관측 중 시뮬레이션 진행 시간 초과")
                rclpy.spin_once(node, timeout_sec=.01)
                current = sim_time()
                speed = state["odom"].twist.twist.linear.x
                if not speed_trace or current > speed_trace[-1][0]:
                    speed_trace.append((current, speed))
                if abs(speed) < .01:
                    if stable_since is None:
                        stable_since = current
                    elif current - stable_since >= .5:
                        stop_stable = True
                        break
                else:
                    stable_since = None
            record_poses = False
            final = state["world_pose"]["position"]
            travel = (final["x"] - initial["x"]) * c + (final["y"] - initial["y"]) * s
            heights = [entry["position"].get("z", 0) for entry in pose_history]
            (output / (trial_name + "_motion_trace.json")).write_text(json.dumps({"world_poses": pose_history, "stop_odometry": speed_trace}), encoding="utf-8")
            last_positions = [entry["position"] for entry in pose_history if entry["sim_stamp_s"] > pose_history[-1]["sim_stamp_s"] - .5]
            stop_displacement = (math.dist([last_positions[0].get(key, 0) for key in ("x", "y")],
                                          [last_positions[-1].get(key, 0) for key in ("x", "y")])
                                 if len(last_positions) > 1 else None)
            if args.flat_control:
                physical_pass = 1.0 < travel < 1.4 and max(heights) - initial_z < .0015
            else:
                physical_pass = (travel > 1.0 and .002 < max(heights) - initial_z < .035
                                 and abs(final.get("z", 0) - initial_z) < .0015 and yellow_pixels > 100)
            report[trial_name] = {
                "image": str(file.relative_to(REPO)), "speed_command_mps": .16, "travel_m": travel,
                "initial_base_z_m": initial_z, "maximum_base_z_m": max(heights),
                "final_base_z_m": final.get("z", 0), "world_pose_samples": len(heights),
                "yellow_stripe_pixels": yellow_pixels,
                "passed": physical_pass,
                "scope": "한 대의 0.16 m/s 저속 통과 1회. 고속 주행·실제 서스펜션 검증 아님.",
            }
            tail_speeds = [speed for stamp, speed in speed_trace if stamp > speed_trace[-1][0] - 2]
            report["stop_check"] = {
                "passed": stop_stable, "criterion": "abs(odom_speed) < 0.01 m/s continuously for 0.5 sim seconds, within 6 sim seconds",
                "wait_sim_s": sim_time() - stopped_at, "last_half_second_displacement_m": stop_displacement,
                "final_speed_mps": state["odom"].twist.twist.linear.x,
                "last_two_second_speed_range_mps": [min(tail_speeds), max(tail_speeds)],
            }
            if not stop_stable:
                report["warning"] = "시설 가시성·통과 결과와 별개로 정지 속도 안정성 기준을 통과하지 못했습니다. 평지 대조 시험과 함께 검토해야 합니다."
            print(f"{trial_name}: {report[trial_name]}; stop_check: {report['stop_check']}", flush=True)
        report["passed"] = all(case["passed"] for case in report["cases"])
        if not args.capture_only:
            report["facility_checks_passed"] = None if args.flat_control else report["passed"] and report["bump_traversal"]["passed"]
            report["passed"] = report["passed"] and report[trial_name]["passed"] and report["stop_check"]["passed"]
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                if process.poll() is not None:
                    break
                try:
                    if sig == signal.SIGINT and process is processes[0]:
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                except ProcessLookupError:
                    break
        log.close()
        report["process_exit_codes"] = [process.poll() for process in processes]
        report["shutdown_clean"] = processes[0].poll() == 0 and all(process.poll() in (0, -signal.SIGINT, 130) for process in processes[1:])
        report["process_errors"] = [line for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
                                    if "[ERROR]" in line or "[Err]" in line or "Traceback" in line or "Segmentation fault" in line]
        report["shutdown_clean"] = report["shutdown_clean"] and not report["process_errors"]
        report["passed"] = report["passed"] and report["shutdown_clean"]
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {key: report[key] for key in ("passed", "capture_only", "bump_only", "shutdown_clean", "process_exit_codes", "process_errors")}
    summary.update(cases_passed=sum(case["passed"] for case in report["cases"]), cases_total=len(report["cases"]),
                   report=str(output / "report.json"))
    for key in ("facility_checks_passed", "bump_traversal", "flat_control", "stop_check", "warning", "error", "initial_detected_ids"):
        if key in report:
            summary[key] = report[key]
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
