#!/usr/bin/env python3
"""공식 자료 기반 official 월드의 독립 Gazebo 실행에서 신호 출발·본선 연속 주행을 검증합니다.

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
DEMO_TRACK = "official"
DEMO_WORLD = REPO / "src/arena_gazebo/worlds/it_arena_official"


def early_start_detected(applied_light, drive_speed, autonomy):
    """초록 적용을 확인하기 전의 허가 또는 비영 속도 명령을 보수적으로 찾습니다."""
    if applied_light == "green":
        return False
    return (abs(float(drive_speed)) > .001 or bool(autonomy.get("started")) or
            abs(float(autonomy.get("speed_command_mps", 0.0))) > .001)


def add_rate_metrics(report, trace, scans, scan, requested_rate_hz):
    """성공·이탈 어느 경우에도 같은 라이다/조향 지표를 보고서에 남깁니다."""
    if trace:
        report["peak_ground_speed_mps"] = max(
            abs(row["dynamics"]["truth_longitudinal_mps"]) for row in trace
        )
        active = [row for row in trace if row["autonomy"].get("started")]
        if active:
            steering = np.asarray([
                row["autonomy"].get("steering_command_rad", 0.0) for row in active
            ])
            errors = np.asarray([row["centerline_error_m"] for row in active])
            speeds = np.asarray([
                abs(row["dynamics"]["truth_longitudinal_mps"]) for row in active
            ])
            report["tracking_metrics"] = {
                "steering_measurement": "약 5 Hz 자율주행 상태 메시지 표본; 원시 조향 명령 아님",
                "centerline_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
                "centerline_p95_m": float(np.percentile(errors, 95)),
                "steering_rms_rad": float(np.sqrt(np.mean(steering ** 2))),
                "steering_max_step_rad": (
                    float(np.max(np.abs(np.diff(steering)))) if len(steering) > 1 else 0.0
                ),
                "steering_total_variation_rad": (
                    float(np.sum(np.abs(np.diff(steering)))) if len(steering) > 1 else 0.0
                ),
                "median_ground_speed_mps": float(np.median(speeds)),
                "p95_ground_speed_mps": float(np.percentile(speeds, 95)),
                "sensor_stop_samples": sum(
                    row["autonomy"].get("state") == "SENSOR_STOP" for row in active
                ),
            }
    if scan is None:
        return
    intervals = np.diff(np.asarray(scans, dtype=float))
    measured_rate = (
        (len(scans) - 1) / (scans[-1] - scans[0])
        if len(scans) > 1 and scans[-1] > scans[0]
        else 0.0
    )
    report["lidar"] = {
        "frame_id": scan.header.frame_id,
        "samples": len(scan.ranges),
        "angle_increment_rad": scan.angle_increment,
        "range_min": scan.range_min,
        "range_max": scan.range_max,
        "scan_time": scan.scan_time,
        "time_increment": scan.time_increment,
        "measured_rate_sim_hz": measured_rate,
        "median_interval_sim_s": float(np.median(intervals)) if len(intervals) else None,
        "max_interval_sim_s": float(np.max(intervals)) if len(intervals) else None,
        "distance_per_scan_at_peak_speed_m": (
            report.get("peak_ground_speed_mps", 0.0) / measured_rate if measured_rate > 0 else None
        ),
        "distance_per_scan_at_20kmh_m": (
            (20 / 3.6) / measured_rate if measured_rate > 0 else None
        ),
        "idealized_snapshot_model": scan.scan_time == 0.0 and scan.time_increment == 0.0,
    }
    report["lidar_rate_passed"] = abs(measured_rate - requested_rate_hz) <= max(
        .15, requested_rate_hz * .08
    )
    report["lidar_nominal_interface_passed"] = (
        scan.header.frame_id == "laser_frame"
        and len(scan.ranges) == 500
        and math.isclose(math.degrees(scan.angle_increment), .72, abs_tol=1e-5)
        and math.isclose(scan.range_min, .05, abs_tol=1e-6)
        and math.isclose(scan.range_max, 12., abs_tol=1e-6)
        and report["lidar_rate_passed"]
    )


def add_depth_metrics(report, stamps, image, info):
    """D435i급 깊이 입력의 표현·명목 주기·내부 파라미터를 별도로 기록합니다."""
    if image is None or info is None:
        report["depth_nominal_interface_passed"] = False
        return
    intervals = np.diff(np.asarray(stamps, dtype=float))
    measured_rate = float((len(stamps) - 1) / (stamps[-1] - stamps[0])
                          if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0)
    report["depth"] = {
        "encoding": image.encoding,
        "width": image.width,
        "height": image.height,
        "frame_id": image.header.frame_id,
        "camera_info_frame_id": info.header.frame_id,
        "camera_info_k": list(info.k),
        "measured_rate_sim_hz": measured_rate,
        "median_interval_sim_s": float(np.median(intervals)) if len(intervals) else None,
        "max_interval_sim_s": float(np.max(intervals)) if len(intervals) else None,
        "simulation_model": "ideal_depth_camera_not_stereo_matching_or_real_D435i_error",
    }
    report["depth_interface_geometry_passed"] = bool(
        image.encoding == "32FC1" and image.width == 848 and image.height == 480 and
        info.width == image.width and info.height == image.height and
        info.k[0] > 0 and info.k[4] > 0
    )
    # Gazebo가 요청한 30 Hz는 명목 설정이고, 실제 전달률은 렌더 부하에 따라
    # 약간 낮아질 수 있습니다. 형상/내부 파라미터 검사와 전달 성능을 분리하여
    # 26.7 Hz를 'D435i 인터페이스 불일치'로 잘못 부르지 않습니다.
    report["depth_delivery_rate_acceptable"] = bool(25.0 <= measured_rate <= 33.0)
    report["depth_nominal_interface_passed"] = (
        report["depth_interface_geometry_passed"] and
        report["depth_delivery_rate_acceptable"]
    )


def start_demo_process(log, speed_profile="cautious", lidar_rate_hz=10.0, tof_safety=True,
                       red_duration_s=8.0, autonomy_mode="lidar", chase_camera=False):
    # 이 검사기는 터미널 프로세스 그룹이 아니라 ros2 launch에만 SIGINT를
    # 보냅니다. WSL의 TTY 상속 여부와 무관하게 자식에게도 전달되게 합니다.
    return subprocess.Popen(
        ["ros2", "launch", "--noninteractive", "arena_bringup", "demo.launch.py", "headless:=true",
         f"speed_profile:={speed_profile}", f"lidar_rate_hz:={lidar_rate_hz:g}",
         f"tof_safety:={str(bool(tof_safety)).lower()}", f"red_duration_s:={red_duration_s:g}",
         f"autonomy_mode:={autonomy_mode}",
         f"chase_camera:={str(bool(chase_camera)).lower()}"],
        cwd=REPO, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=2)
    parser.add_argument("--max-sim-seconds", type=float, default=900)
    parser.add_argument("--speed-profile", choices=("cautious", "brisk", "exploratory", "hardware_target", "lidar_rate_stress", "lidar_20kmh_straight"), default="cautious")
    parser.add_argument("--lidar-rate-hz", type=float, default=10.0,
                        help="Gazebo 이상화 라이다의 시험용 갱신률. C1 실물 회전수를 바꾸지 않습니다.")
    parser.add_argument("--target-progress-m", type=float,
                        help="한 바퀴 대신 지정 거리까지만 검사합니다. 20 km/h 직선 스트레스 등에 사용합니다.")
    parser.add_argument("--disable-tof-safety", action="store_true",
                        help="라이다 조향 주기만 분리해 볼 때 ToF 속도 제한을 끕니다. 안전 검증이 아닙니다.")
    parser.add_argument("--red-duration-s", type=float, default=8.0,
                        help="검사 시작 전 빨간 신호 유지 시간. 기본 데모와 같은 8초입니다.")
    parser.add_argument("--autonomy-mode", choices=("lidar", "stereo"), default="lidar",
                        help="lidar=C1+ToF6, stereo=전방 D435i 깊이+측면 ToF4")
    parser.add_argument("--video", action="store_true",
                        help="제어 입력과 분리된 차량 추적 카메라를 H.264 MP4로 기록합니다.")
    parser.add_argument("--output", type=Path, default=REPO / "artifacts/tests/basic_autonomy_official",
                        help="official 검사 출력 폴더. 이전 실험 트랙 기록과 분리합니다.")
    args = parser.parse_args()
    if (args.laps < 1 or not math.isfinite(args.max_sim_seconds) or args.max_sim_seconds <= 0 or
            not math.isfinite(args.lidar_rate_hz) or not .5 <= args.lidar_rate_hz <= 100 or
            not math.isfinite(args.red_duration_s) or not .5 <= args.red_duration_s <= 30 or
            (args.target_progress_m is not None and
             (not math.isfinite(args.target_progress_m) or args.target_progress_m <= 0))):
        raise ValueError("바퀴 수는 1 이상, 시간 한도는 유한한 양수여야 합니다.")
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("검사 출력은 프로젝트 artifacts 아래에 둡니다.")
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(180 + os.getpid() % 30), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                      ROS_STATIC_PEERS="", GZ_PARTITION=f"arena_autonomy_test_{os.getpid()}")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("basic_autonomy_validator")
    state, trace, scans, depth_stamps, pictures = {}, [], [], [], set()
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
        node.create_subscription(Odometry, "/odom", lambda m: remember(m, "odom"), 10),
        node.create_subscription(AckermannDriveStamped, "/drive", lambda m: remember(m, "drive"), 10),
        node.create_subscription(String, "/autonomy/status", lambda m: remember(json.loads(m.data), "autonomy"), 10),
        node.create_subscription(String, "/sim/traffic_light/state", lambda m: remember(json.loads(m.data), "light"), 10),
        node.create_subscription(String, "/sim/drivetrain", lambda m: remember(json.loads(m.data), "dynamics"), 10),
        node.create_subscription(String, "/safety/status", lambda m: remember(json.loads(m.data), "safety"), 10),
        node.create_subscription(AckermannDriveStamped, "/drive/safe", lambda m: remember(m, "safe_drive"), 10),
    ]
    if args.autonomy_mode == "lidar":
        subscriptions.append(node.create_subscription(LaserScan, "/scan", receive_scan, qos_profile_sensor_data))
    else:
        def receive_depth(message):
            state["depth"] = message
            depth_stamps.append(float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9)
        subscriptions.extend([
            node.create_subscription(Image, "/camera/depth/image_rect_raw", receive_depth, qos_profile_sensor_data),
            node.create_subscription(CameraInfo, "/camera/depth/camera_info", lambda m: remember(m, "depth_info"), qos_profile_sensor_data),
        ])
    if args.video:
        subscriptions.append(node.create_subscription(
            Image, "/sim/chase/image", lambda m: remember(m, "chase"), QoSProfile(depth=2)
        ))
    stop_service = node.create_client(SetBool, "/autonomy/enable")
    # demo.launch.py의 track=official과 같은 월드로 진행 거리와 충돌을 판정합니다.
    world = DEMO_WORLD
    route = np.genfromtxt(world / "centerline.csv", delimiter=",", names=True)
    xy = np.column_stack((route["x_m"], route["y_m"]))
    scene = json.loads((world / "scene.json").read_text())
    lap_length = float(scene["track"]["lap_length_m"])
    target_progress = float(args.target_progress_m if args.target_progress_m is not None
                            else args.laps * lap_length + 2)
    full_lap_validation = target_progress >= lap_length
    obstacles = obstacle_polygons(world / "world.sdf")
    tree = STRtree(obstacles)
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "passed": False, "track": DEMO_TRACK,
              "requested_laps": args.laps, "target_progress_m": target_progress,
              "full_lap_validation": full_lap_validation, "lap_length_m": lap_length,
              "speed_profile": args.speed_profile, "requested_lidar_rate_hz": args.lidar_rate_hz,
              "autonomy_mode": args.autonomy_mode,
              "sensor_layout": "lidar_tof6" if args.autonomy_mode == "lidar" else "stereo_tof4",
              "simulated_total_mass_kg": 2.0,
              "comparison_ballast_kg": 0.0 if args.autonomy_mode == "lidar" else 0.111,
              "mass_comparison_scope": "same 2.000 kg total mass; removed sensor mass added to chassis for A/B; CG/inertia distribution is not identical; recording camera adds 0.000001 kg",
              "video_requested": args.video,
              "tof_safety_enabled": not args.disable_tof_safety,
              "red_duration_s": args.red_duration_s,
              "validation_scope": "requested progress and stop; speed-profile limit is not a verified target speed",
              "collision_scope": "active path samples only; stop-phase collision is not assessed by this validator",
              "input_sha256": {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in [Path(__file__).resolve(), world / "world.sdf", REPO / "src/arena_description/config/vehicle.yaml",
                                            REPO / "src/arena_description/models/arena_car/model.sdf.xacro",
                                            REPO / "src/arena_autonomy/arena_autonomy/core.py",
                                            REPO / "src/arena_autonomy/arena_autonomy/wall_follow.py",
                                            REPO / "src/arena_autonomy/arena_autonomy/stereo_wall_follow.py",
                                            REPO / "src/arena_autonomy/arena_autonomy/stereo_road.py",
                                            REPO / "src/arena_gazebo/scripts/traffic_light_controller.py",
                                            REPO / "src/arena_bringup/launch/demo.launch.py",
                                            REPO / "src/arena_bringup/launch/simulation.launch.py",
                                            REPO / "src/arena_gazebo/src/single_motor_drive.cpp",
                                            REPO / "src/arena_gazebo/include/arena_gazebo/drivetrain.hpp",
                                            REPO / "src/arena_vehicle_interface/arena_vehicle_interface/sim_wheel_encoder.py",
                                            REPO / "src/arena_vehicle_interface/arena_vehicle_interface/ackermann_to_twist.py",
                                            REPO / "src/arena_bringup/config/tof_safety.yaml",
                                            REPO / "src/arena_autonomy/arena_autonomy/tof_safety.py",
                                            REPO / "src/arena_autonomy/arena_autonomy/tof_safety_core.py",
                                            REPO / "src/arena_bringup/config/wall_follow.yaml"]}}
    processes = []
    log_path = output / "simulation.log"
    log = log_path.open("w", encoding="utf-8")
    checkpoint_path = output / "progress.json"
    trace_stream = (output / "trajectory.jsonl").open("w", encoding="utf-8")
    # 파일명 자체가 실패·단기 실행을 완주처럼 표현하지 않게 합니다.
    video_path = output / f"{args.autonomy_mode}_third_person_drive.mp4"
    video_writer = None
    video_frames = 0
    first_video_stamp = None
    last_video_stamp = -math.inf
    def checkpoint(payload):
        temporary = output / "progress.tmp"
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(checkpoint_path)
        trace_stream.flush()
    checkpoint({"completed": False, "state": "STARTING", "pid": os.getpid(),
                "started_at_utc": report["started_at_utc"], "input_sha256": report["input_sha256"]})
    try:
        processes.append(start_demo_process(log, args.speed_profile, args.lidar_rate_hz,
                                            not args.disable_tof_safety, args.red_duration_s,
                                            args.autonomy_mode, args.video))
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
        run_started, stalled_since, drivetrain_stalled_since = None, None, None
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
            required = {"pose", "autonomy", "light", "rgb", "odom", "drive", "dynamics"}
            required.add("scan" if args.autonomy_mode == "lidar" else "depth")
            if args.autonomy_mode == "stereo":
                required.add("depth_info")
            if args.video:
                required.add("chase")
            if not args.disable_tof_safety:
                required.update(("safety", "safe_drive"))
            if not required.issubset(state):
                if time.monotonic() - began > 90:
                    raise TimeoutError(f"시작 데이터 미수신: {sorted(state)}")
                continue
            pose = state["pose"]
            now = pose["time"]
            if now - last_record < (.045 if args.video else .10):
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
            if not args.disable_tof_safety:
                false_start |= early_start_detected(light["applied"], state["safe_drive"].drive.speed, autonomous)
            if autonomous["started"] and run_started is None:
                run_started = now
                report["start_time_sim_s"] = now
            if args.video and run_started is not None:
                chase = state["chase"]
                chase_stamp = float(chase.header.stamp.sec) + chase.header.stamp.nanosec * 1e-9
                if chase_stamp > last_video_stamp:
                    frame = cv2.cvtColor(image_array(chase), cv2.COLOR_RGB2BGR)
                    if video_writer is None:
                        video_writer = cv2.VideoWriter(
                            str(video_path), cv2.VideoWriter_fourcc(*"avc1"), 20.0,
                            (frame.shape[1], frame.shape[0]),
                        )
                        if not video_writer.isOpened():
                            raise RuntimeError("H.264 주행 영상 인코더를 열지 못했습니다.")
                        cv2.imwrite(str(output / "third_person_first_frame.png"), frame)
                    if first_video_stamp is None:
                        first_video_stamp = chase_stamp
                    label = "C1 LiDAR + ToF 6" if args.autonomy_mode == "lidar" else "D435i RGB-D + ToF 4"
                    cv2.rectangle(frame, (18, 16), (535, 83), (12, 16, 22), -1)
                    cv2.putText(frame, label, (32, 45), cv2.FONT_HERSHEY_SIMPLEX,
                                .72, (245, 245, 245), 2, cv2.LINE_AA)
                    cv2.putText(frame, f"progress {progress:5.1f} m   speed {abs(state['dynamics']['truth_longitudinal_mps']):.2f} m/s",
                                (32, 72), cv2.FONT_HERSHEY_SIMPLEX, .55,
                                (120, 220, 255), 1, cv2.LINE_AA)
                    video_writer.write(frame)
                    video_frames += 1
                    last_video_stamp = chase_stamp
            record = {"time": now, "x": float(position[0]), "y": float(position[1]), "s": s,
                      "progress_m": progress, "centerline_error_m": error, "collision": collision,
                      "yaw": yaw_of(pose["orientation"]), "autonomy": autonomous, "light": light["applied"],
                      "odom_speed": state["odom"].twist.twist.linear.x,
                      "dynamics": state["dynamics"], "safety": state.get("safety"),
                      "safe_drive_speed_mps": (state["safe_drive"].drive.speed
                                               if "safe_drive" in state else None)}
            trace.append(record)
            trace_stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            for label in [f"signal_{light['applied']}", *[f"marker_{value}" for value in autonomous["marker_ids"]]]:
                if label not in pictures and label != "signal_unknown":
                    cv2.imwrite(str(output / f"{label}.png"), cv2.cvtColor(image_array(state["rgb"]), cv2.COLOR_RGB2BGR))
                    pictures.add(label)
            if time.monotonic() - last_progress_log > 15:
                print(f"sim={now:.1f}s progress={progress:.2f}/{target_progress:.2f}m "
                      f"s={s:.2f} error={error:.3f} {autonomous['state']} {autonomous['side']} "
                      f"light={light['applied']} markers={sorted(markers_seen)}", flush=True)
                checkpoint({"completed": False, "pid": os.getpid(), "sim_time_s": now,
                            "progress_m": progress, "target_progress_m": target_progress,
                            "completed_laps": int(max(0, progress) // lap_length),
                            "state": autonomous["state"], "light": light["applied"],
                            "marker_ids": sorted(markers_seen), "collision_samples": collision_samples,
                            "max_centerline_error_m": max_error})
                last_progress_log = time.monotonic()
            safety_stopped = (not args.disable_tof_safety and state["safety"]["safe_speed_mps"] == 0)
            if run_started is not None and (autonomous["state"] != "RUNNING" or safety_stopped):
                stalled_since = now if stalled_since is None else stalled_since
                if now - stalled_since > 7:
                    raise RuntimeError(
                        f"주행 정지 상태 지속: autonomy={autonomous}, safety={state.get('safety')}"
                    )
            else:
                stalled_since = None
            dynamics = state["dynamics"]
            safe_command = abs(float(state["safe_drive"].drive.speed)
                               if "safe_drive" in state else float(state["drive"].drive.speed))
            truth_speed = math.hypot(float(dynamics["truth_longitudinal_mps"]),
                                     float(dynamics["truth_lateral_mps"]))
            wheel_speed = abs(float(dynamics["wheel_surface_speed_mps"]))
            drivetrain_stalled = (
                run_started is not None and autonomous["state"] == "RUNNING" and
                safe_command > .12 and wheel_speed > .12 and truth_speed < .01
            )
            if drivetrain_stalled:
                drivetrain_stalled_since = (now if drivetrain_stalled_since is None
                                             else drivetrain_stalled_since)
                if now - drivetrain_stalled_since > 3.0:
                    report["drivetrain_stall"] = {
                        "duration_sim_s": now - drivetrain_stalled_since,
                        "safe_command_mps": safe_command,
                        "truth_speed_mps": truth_speed,
                        "wheel_surface_speed_mps": wheel_speed,
                        "left_speed_rad_s": float(dynamics["left_speed_rad_s"]),
                        "right_speed_rad_s": float(dynamics["right_speed_rad_s"]),
                        "interpretation": "commanded motion with wheel rotation but near-zero body motion",
                    }
                    raise RuntimeError(
                        "구동 정지/헛돎 지속: 명령과 바퀴 회전은 있으나 차체가 3초 이상 이동하지 않음"
                    )
            else:
                drivetrain_stalled_since = None
            if collision_samples > 3 or error > .35:
                raise RuntimeError(f"벽 접촉 또는 본선 이탈: collision_samples={collision_samples}, error={error:.3f}")
            if progress >= target_progress:
                break
            if now - first_pose_time > args.max_sim_seconds:
                raise TimeoutError("설정한 시뮬레이션 시간 안에 연속 주행을 완료하지 못했습니다.")
        report.update(progress_m=progress, completed_laps=int(progress // lap_length),
                      max_centerline_error_m=max_error, collision_samples=collision_samples,
                      false_start=false_start, signal_states=sorted(signal_states), marker_ids=sorted(markers_seen),
                      wall_sides=sorted(sides_seen))
        controller_name = "wall_follow" if args.autonomy_mode == "lidar" else "stereo_wall_follow"
        inputs = {topic for topic, _ in node.get_subscriber_names_and_types_by_node(controller_name, "/")}
        report["autonomy_subscriptions"] = sorted(inputs)
        required_inputs = ({"/scan", "/camera/color/image_raw"} if args.autonomy_mode == "lidar" else
                           {"/camera/depth/image_rect_raw", "/camera/depth/camera_info",
                            "/camera/color/image_raw"})
        allowed_inputs = required_inputs | {"/clock", "/parameter_events"}
        report["sensor_only_inputs"] = required_inputs.issubset(inputs) and inputs.issubset(allowed_inputs)
        if args.disable_tof_safety:
            report["safety_subscriptions"] = []
            report["safety_sensor_only_inputs"] = None
        else:
            safety_inputs = {topic for topic, _ in node.get_subscriber_names_and_types_by_node("tof_safety", "/")}
            active_tof_names = (("front", "front_left", "rear_left", "rear", "rear_right", "front_right")
                                if args.autonomy_mode == "lidar" else
                                ("front_left", "rear_left", "rear_right", "front_right"))
            tof_inputs = {f"/tof/{name}/points" for name in active_tof_names}
            report["safety_subscriptions"] = sorted(safety_inputs)
            report["safety_sensor_only_inputs"] = (tof_inputs | {"/drive", "/wheel_states"}).issubset(safety_inputs) and safety_inputs.issubset(
                tof_inputs | {"/drive", "/wheel_states", "/clock", "/parameter_events"})
        add_rate_metrics(report, trace, scans, state.get("scan"), args.lidar_rate_hz)
        if args.autonomy_mode == "lidar":
            source_interface_passed = report["lidar_nominal_interface_passed"]
        else:
            add_depth_metrics(report, depth_stamps, state.get("depth"), state.get("depth_info"))
            source_interface_passed = report["depth_nominal_interface_passed"]
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
            if (abs(state["odom"].twist.twist.linear.x) < .01 and
                    abs(state["dynamics"]["truth_longitudinal_mps"]) < .01 and
                    abs(state["dynamics"]["truth_lateral_mps"]) < .01):
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
        route_evidence = ({20, 30}.issubset(markers_seen) if full_lap_validation else progress >= target_progress)
        safety_evidence = (True if args.disable_tof_safety else report["safety_sensor_only_inputs"])
        report["passed"] = (not false_start and collision_samples == 0 and report["stop_stable"] and
                            report["sensor_only_inputs"] and safety_evidence and source_interface_passed and
                            {"red", "yellow", "green"}.issubset(signal_states) and route_evidence)
    except Exception as error:
        report["error"] = str(error)
        print(f"검사 실패: {error}", flush=True)
    finally:
        if "tracking_metrics" not in report:
            add_rate_metrics(report, trace, scans, state.get("scan"), args.lidar_rate_hz)
        if args.autonomy_mode == "stereo" and "depth" not in report:
            add_depth_metrics(report, depth_stamps, state.get("depth"), state.get("depth_info"))
        if "rgb" in state:
            cv2.imwrite(str(output / "last_rgb.png"), cv2.cvtColor(image_array(state["rgb"]), cv2.COLOR_RGB2BGR))
        if "scan" in state:
            scan = state["scan"]
            (output / "last_scan.json").write_text(json.dumps({"ranges": [float(r) if math.isfinite(r) else None for r in scan.ranges], "angle_min": scan.angle_min,
                "angle_increment": scan.angle_increment, "range_min": scan.range_min, "range_max": scan.range_max,
                "frame_id": scan.header.frame_id, "scan_time": scan.scan_time, "time_increment": scan.time_increment}))
        if "depth" in state:
            depth_message = state["depth"]
            dtype = np.dtype(">f4" if depth_message.is_bigendian else "<f4")
            values = np.frombuffer(depth_message.data, dtype=dtype).reshape(
                depth_message.height, depth_message.step // 4)[:, :depth_message.width]
            depth_mm = np.clip(np.nan_to_num(values, nan=0.0, posinf=0.0, neginf=0.0) * 1000, 0, 65535).astype(np.uint16)
            cv2.imwrite(str(output / "last_depth_mm.png"), depth_mm)
        if video_writer is not None:
            video_writer.release()
            report["video"] = {
                "path": str(video_path.relative_to(REPO)),
                "codec": "H.264/avc1",
                "fps": 20.0,
                "frames_written": video_frames,
                "duration_s": video_frames / 20.0,
                "first_frame_sim_time_s": first_video_stamp,
                "last_frame_sim_time_s": last_video_stamp,
                "captured_span_sim_s": last_video_stamp - first_video_stamp,
                "timing_scope": "20 fps of captured simulation frames, not wall-clock real-time performance",
                "sha256": hashlib.sha256(video_path.read_bytes()).hexdigest(),
                "control_input": False,
            }
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
        if args.video:
            capture = cv2.VideoCapture(str(video_path))
            declared_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) if capture.isOpened() else 0
            decoded_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) if capture.isOpened() else 0
            decoded_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) if capture.isOpened() else 0
            decoded_frames, invalid_dimensions = 0, False
            while capture.isOpened():
                valid, frame = capture.read()
                if not valid:
                    break
                decoded_frames += 1
                invalid_dimensions |= frame.shape[:2] != (540, 960)
            capture.release()
            report.setdefault("video", {}).update(
                declared_frames=declared_frames, decoded_frames=decoded_frames,
                width=decoded_width, height=decoded_height,
                decode_passed=(declared_frames == decoded_frames == video_frames and
                               decoded_frames > 0 and not invalid_dimensions and
                               (decoded_width, decoded_height) == (960, 540)),
            )
            report["passed"] = report["passed"] and report["video"]["decode_passed"]
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
