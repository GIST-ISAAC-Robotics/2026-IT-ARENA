#!/usr/bin/env python3
"""독립 Gazebo 실행에서 신호 출발·본선 연속 주행을 검증합니다.

정답 위치·중심선은 이 검사기의 판정에만 사용하며 자율주행 노드에 전달하지 않습니다.
"""
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
from shapely.strtree import STRtree

from build_experimental_track import obstacle_polygons, rectangle
from validate_facility_visibility import image_array, yaw_of

REPO = Path(__file__).resolve().parents[1]


def early_start_detected(applied_light, drive_speed, autonomy):
    """초록 적용을 확인하기 전의 허가 또는 비영 속도 명령을 보수적으로 찾습니다."""
    if applied_light == "green":
        return False
    return (abs(float(drive_speed)) > .001 or bool(autonomy.get("started")) or
            abs(float(autonomy.get("speed_command_mps", 0.0))) > .001)


def start_demo_process(log):
    # 이 검사기는 터미널 프로세스 그룹이 아니라 ros2 launch에만 SIGINT를
    # 보냅니다. WSL의 TTY 상속 여부와 무관하게 자식에게도 전달되게 합니다.
    return subprocess.Popen(
        ["ros2", "launch", "--noninteractive", "arena_bringup", "demo.launch.py", "headless:=true"],
        cwd=REPO, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=2)
    parser.add_argument("--max-sim-seconds", type=float, default=900)
    parser.add_argument("--output", type=Path, default=REPO / "artifacts/tests/basic_autonomy")
    args = parser.parse_args()
    if args.laps < 1 or not math.isfinite(args.max_sim_seconds) or args.max_sim_seconds <= 0:
        raise ValueError("바퀴 수는 1 이상, 시간 한도는 유한한 양수여야 합니다.")
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("검사 출력은 프로젝트 artifacts 아래에 둡니다.")
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(180 + os.getpid() % 30), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                      ROS_STATIC_PEERS="", GZ_PARTITION=f"arena_autonomy_test_{os.getpid()}")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("basic_autonomy_validator")
    state, trace, scans, pictures = {}, [], [], set()
    stop_requested = False
    def interrupted(*_):
        nonlocal stop_requested
        stop_requested = True
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    def remember(message, key):
        state[key] = message
    def receive_scan(message):
        state["scan"] = message
        scans.append(float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9)
    subscriptions = [
        node.create_subscription(Image, "/camera/color/image_raw", lambda m: remember(m, "rgb"), QoSProfile(depth=2)),
        node.create_subscription(LaserScan, "/scan", receive_scan, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", lambda m: remember(m, "odom"), 10),
        node.create_subscription(AckermannDriveStamped, "/drive", lambda m: remember(m, "drive"), 10),
        node.create_subscription(String, "/autonomy/status", lambda m: remember(json.loads(m.data), "autonomy"), 10),
        node.create_subscription(String, "/sim/traffic_light/state", lambda m: remember(json.loads(m.data), "light"), 10),
    ]
    stop_service = node.create_client(SetBool, "/autonomy/enable")
    world = REPO / "src/arena_gazebo/worlds/it_arena_experimental"
    route = np.genfromtxt(world / "centerline.csv", delimiter=",", names=True)
    xy = np.column_stack((route["x_m"], route["y_m"]))
    scene = json.loads((world / "scene.json").read_text())
    lap_length = float(scene["track"]["lap_length_m"])
    obstacles = obstacle_polygons(world / "world.sdf")
    tree = STRtree(obstacles)
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "passed": False,
              "requested_laps": args.laps, "lap_length_m": lap_length,
              "input_sha256": {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in [world / "world.sdf", REPO / "src/arena_description/config/vehicle.yaml",
                                            REPO / "src/arena_description/models/arena_car/model.sdf.xacro",
                                            REPO / "src/arena_autonomy/arena_autonomy/core.py",
                                            REPO / "src/arena_autonomy/arena_autonomy/wall_follow.py",
                                            REPO / "src/arena_gazebo/scripts/traffic_light_controller.py",
                                            REPO / "src/arena_bringup/launch/demo.launch.py",
                                            REPO / "src/arena_bringup/launch/simulation.launch.py",
                                            REPO / "src/arena_bringup/config/wall_follow.yaml"]}}
    processes = []
    log_path = output / "simulation.log"
    log = log_path.open("w", encoding="utf-8")
    checkpoint_path = output / "progress.json"
    trace_stream = (output / "trajectory.jsonl").open("w", encoding="utf-8")
    def checkpoint(payload):
        temporary = output / "progress.tmp"
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(checkpoint_path)
        trace_stream.flush()
    checkpoint({"completed": False, "state": "STARTING", "pid": os.getpid(),
                "started_at_utc": report["started_at_utc"], "input_sha256": report["input_sha256"]})
    try:
        processes.append(start_demo_process(log))
        pose_process = subprocess.Popen(["gz", "topic", "-e", "-t", "/world/it_arena_track/dynamic_pose/info", "--json-output"],
                                        cwd=REPO, stdout=subprocess.PIPE, stderr=log, text=True, start_new_session=True)
        processes.append(pose_process)
        def observe_poses():
            for line in pose_process.stdout:
                try:
                    data = json.loads(line)
                    stamp = data.get("header", {}).get("stamp", {})
                    for pose in data.get("pose", []):
                        if pose.get("name") == "arena_car":
                            pose["time"] = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9
                            state["pose"] = pose
                except (ValueError, TypeError):
                    continue
        threading.Thread(target=observe_poses, daemon=True).start()
        began = time.monotonic()
        last_record, last_progress_log = -math.inf, 0
        last_s, progress, max_error = None, 0.0, 0.0
        first_pose_time = None
        run_started, stalled_since = None, None
        signal_states, markers_seen, sides_seen = set(), set(), set()
        false_start = False
        collision_samples = 0
        while True:
            rclpy.spin_once(node, timeout_sec=.005)
            if stop_requested:
                raise RuntimeError("검사 중단 요청")
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("검사 프로세스가 예상보다 일찍 종료했습니다.")
            if time.monotonic() - began > max(180, args.max_sim_seconds * 7):
                raise TimeoutError("실제 시간 검사 한도 초과")
            if not all(key in state for key in ("pose", "autonomy", "light", "rgb", "scan", "odom", "drive")):
                if time.monotonic() - began > 90:
                    raise TimeoutError(f"시작 데이터 미수신: {sorted(state)}")
                continue
            pose = state["pose"]
            now = pose["time"]
            if now - last_record < .10:
                continue
            last_record = now
            first_pose_time = now if first_pose_time is None else first_pose_time
            position = np.array([pose["position"]["x"], pose["position"]["y"]])
            index = int(np.argmin(np.sum((xy - position) ** 2, axis=1)))
            s = float(route["s_m"][index])
            if last_s is not None:
                delta = (s - last_s + lap_length / 2) % lap_length - lap_length / 2
                if abs(delta) > 1:
                    raise RuntimeError(f"본선 진행 위치의 비연속 점프: {delta:.3f} m")
                progress += delta
            last_s = s
            error = float(np.linalg.norm(xy[index] - position))
            max_error = max(error, max_error)
            footprint = rectangle(*position, yaw_of(pose["orientation"]), .20, .15)
            collision = any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))
            collision_samples += int(collision)
            autonomous, light = state["autonomy"], state["light"]
            signal_states.add(light["applied"])
            markers_seen.update(autonomous["marker_ids"])
            sides_seen.add(autonomous["side"])
            false_start |= early_start_detected(light["applied"], state["drive"].drive.speed, autonomous)
            if autonomous["started"] and run_started is None:
                run_started = now
                report["start_time_sim_s"] = now
            record = {"time": now, "x": float(position[0]), "y": float(position[1]), "s": s,
                      "progress_m": progress, "centerline_error_m": error, "collision": collision,
                      "yaw": yaw_of(pose["orientation"]), "autonomy": autonomous, "light": light["applied"],
                      "odom_speed": state["odom"].twist.twist.linear.x}
            trace.append(record)
            trace_stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            for label in [f"signal_{light['applied']}", *[f"marker_{value}" for value in autonomous["marker_ids"]]]:
                if label not in pictures and label != "signal_unknown":
                    cv2.imwrite(str(output / f"{label}.png"), cv2.cvtColor(image_array(state["rgb"]), cv2.COLOR_RGB2BGR))
                    pictures.add(label)
            if time.monotonic() - last_progress_log > 15:
                print(f"sim={now:.1f}s progress={progress:.2f}/{args.laps * lap_length + 2:.2f}m "
                      f"s={s:.2f} error={error:.3f} {autonomous['state']} {autonomous['side']} "
                      f"light={light['applied']} markers={sorted(markers_seen)}", flush=True)
                checkpoint({"completed": False, "pid": os.getpid(), "sim_time_s": now,
                            "progress_m": progress, "target_progress_m": args.laps * lap_length + 2,
                            "completed_laps": int(max(0, progress) // lap_length),
                            "state": autonomous["state"], "light": light["applied"],
                            "marker_ids": sorted(markers_seen), "collision_samples": collision_samples,
                            "max_centerline_error_m": max_error})
                last_progress_log = time.monotonic()
            if run_started is not None and autonomous["state"] != "RUNNING":
                stalled_since = now if stalled_since is None else stalled_since
                if now - stalled_since > 7:
                    raise RuntimeError(f"주행 정지 상태 지속: {autonomous}")
            else:
                stalled_since = None
            if collision_samples > 3 or error > .35:
                raise RuntimeError(f"벽 접촉 또는 본선 이탈: collision_samples={collision_samples}, error={error:.3f}")
            if progress >= args.laps * lap_length + 2:
                break
            if now - first_pose_time > args.max_sim_seconds:
                raise TimeoutError("설정한 시뮬레이션 시간 안에 연속 주행을 완료하지 못했습니다.")
        report.update(progress_m=progress, completed_laps=int(progress // lap_length),
                      max_centerline_error_m=max_error, collision_samples=collision_samples,
                      false_start=false_start, signal_states=sorted(signal_states), marker_ids=sorted(markers_seen),
                      wall_sides=sorted(sides_seen))
        inputs = {topic for topic, _ in node.get_subscriber_names_and_types_by_node("wall_follow", "/")}
        report["autonomy_subscriptions"] = sorted(inputs)
        report["sensor_only_inputs"] = {"/scan", "/camera/color/image_raw"}.issubset(inputs) and inputs.issubset(
            {"/scan", "/camera/color/image_raw", "/clock", "/parameter_events"})
        scan = state["scan"]
        report["lidar"] = {"frame_id": scan.header.frame_id, "samples": len(scan.ranges),
                           "angle_increment_rad": scan.angle_increment, "range_min": scan.range_min,
                           "range_max": scan.range_max, "scan_time": scan.scan_time,
                           "time_increment": scan.time_increment,
                           "measured_rate_sim_hz": (len(scans) - 1) / (scans[-1] - scans[0])}
        report["lidar_nominal_interface_passed"] = (
            scan.header.frame_id == "laser_frame" and len(scan.ranges) == 500 and
            math.isclose(math.degrees(scan.angle_increment), .72, abs_tol=1e-5) and
            math.isclose(scan.range_min, .05, abs_tol=1e-6) and math.isclose(scan.range_max, 12., abs_tol=1e-6) and
            report["lidar"]["measured_rate_sim_hz"] > 9.0)
        if not stop_service.wait_for_service(timeout_sec=3):
            raise TimeoutError("차량 정지 서비스를 찾지 못했습니다.")
        future = stop_service.call_async(SetBool.Request(data=False))
        stop_start, stable_since = state["pose"]["time"], None
        stop_origin = state["pose"]["position"].copy()
        stop_deadline = time.monotonic() + 60
        while state["pose"]["time"] - stop_start < 6:
            if stop_requested or time.monotonic() > stop_deadline or any(p.poll() is not None for p in processes):
                raise TimeoutError("정지 검사 중 중단·시간 초과·프로세스 종료를 확인했습니다.")
            rclpy.spin_once(node, timeout_sec=.01)
            now = state["pose"]["time"]
            if abs(state["odom"].twist.twist.linear.x) < .01:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= .5 and future.done() and future.result().success:
                    break
            else:
                stable_since = None
        report["stop_stable"] = (stable_since is not None and state["pose"]["time"] - stable_since >= .5 and
                                 future.done() and future.result().success)
        report["stop_wait_sim_s"] = state["pose"]["time"] - stop_start
        report["stop_displacement_m"] = math.dist(
            [stop_origin.get(axis, 0.) for axis in ("x", "y")],
            [state["pose"]["position"].get(axis, 0.) for axis in ("x", "y")])
        report["final_odom_speed_mps"] = state["odom"].twist.twist.linear.x
        report["passed"] = (not false_start and collision_samples == 0 and report["stop_stable"] and
                            report["sensor_only_inputs"] and report["lidar_nominal_interface_passed"] and
                            {"red", "yellow", "green"}.issubset(signal_states) and {20, 30}.issubset(markers_seen))
    except Exception as error:
        report["error"] = str(error)
        print(f"검사 실패: {error}", flush=True)
    finally:
        if "rgb" in state:
            cv2.imwrite(str(output / "last_rgb.png"), cv2.cvtColor(image_array(state["rgb"]), cv2.COLOR_RGB2BGR))
        if "scan" in state:
            scan = state["scan"]
            (output / "last_scan.json").write_text(json.dumps({"ranges": [float(r) if math.isfinite(r) else None for r in scan.ranges], "angle_min": scan.angle_min,
                "angle_increment": scan.angle_increment, "range_min": scan.range_min, "range_max": scan.range_max,
                "frame_id": scan.header.frame_id, "scan_time": scan.scan_time, "time_increment": scan.time_increment}))
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                if process.poll() is not None:
                    break
                try:
                    if process is processes[0] and sig == signal.SIGINT:
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    continue
        log.close()
        report["process_exit_codes"] = [process.poll() for process in processes]
        report["process_errors"] = [line for line in log_path.read_text(errors="replace").splitlines()
                                    if "[ERROR]" in line or "[Err]" in line or "Traceback" in line or "Segmentation fault" in line]
        report["shutdown_clean"] = (bool(processes) and processes[0].poll() == 0 and
                                    all(p.poll() in (0, -signal.SIGINT, 130) for p in processes[1:]) and not report["process_errors"])
        report["passed"] = report["passed"] and report["shutdown_clean"]
        if trace:
            report["last_observation"] = trace[-1]
            report.setdefault("progress_m", trace[-1]["progress_m"])
            report.setdefault("completed_laps", int(max(0, trace[-1]["progress_m"]) // lap_length))
            report.setdefault("max_centerline_error_m", max(row["centerline_error_m"] for row in trace))
            report.setdefault("collision_samples", sum(row["collision"] for row in trace))
        (output / "trace.json").write_text(json.dumps(trace, ensure_ascii=False, indent=2), encoding="utf-8")
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        checkpoint({"completed": True, "passed": report["passed"], "pid": os.getpid(),
                    "progress_m": report.get("progress_m"), "report": str(output / "report.json"),
                    "error": report.get("error")})
        trace_stream.close()
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
