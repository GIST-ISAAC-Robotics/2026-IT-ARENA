#!/usr/bin/env python3
"""선택한 월드에서 짧은 직진·회전·센서·명령 중단 정지를 검증합니다. 완주 시험은 아닙니다."""

from __future__ import annotations

import argparse
from array import array
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", choices=("original", "experimental"), default="experimental")
    parser.add_argument("--d435i-profile", choices=("configured", "high_speed_async", "synchronized_60", "low_load_30"),
                        default="configured")
    parser.add_argument("--tof-profile", choices=("configured", "low_latency_4x4_60", "tracking_8x8_15"),
                        default="configured")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    output = repo / "artifacts/tests"
    output.mkdir(parents=True, exist_ok=True)
    # 다른 사용자의 시뮬레이션과 ROS/Gazebo 명령이 섞이지 않게 격리합니다.
    os.environ["ROS_DOMAIN_ID"] = str(70 + os.getpid() % 100)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["GZ_PARTITION"] = f"it_arena_smoke_{os.getpid()}"
    # 다중 MB 영상에 적합한 전송 모드를 선택합니다. 누락 방지 보장은 아니며
    # 실제 처리율은 아래에서 별도로 측정합니다. 기존 사용자 지정은 유지합니다.
    # 수신 노드와 하위 launch 모두 같은 전송 모드로 시작해야 합니다.
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
    from std_msgs.msg import Int64MultiArray
    import yaml

    config_path = repo / "src/arena_description/config/vehicle.yaml"
    sensor_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["vehicle"]["sensors"]
    camera_config = sensor_config["d435i"]
    tof_config = sensor_config["tof_ring"]
    profile_name = (camera_config["active_stream_profile"] if args.d435i_profile == "configured"
                    else args.d435i_profile)
    profile = camera_config["stream_profiles"][profile_name]
    tof_profile_name = (tof_config["active_profile"] if args.tof_profile == "configured"
                        else args.tof_profile)
    tof_profile = tof_config["profiles"][tof_profile_name]

    rclpy.init()
    node = rclpy.create_node("arena_smoke_test")
    state = {}
    camera_stamps = {"rgb": [], "depth": [], "rgb_info": [], "depth_info": []}
    tof_modules = tof_config["modules"] if tof_config["enabled"] else []
    tof_stamps = {f"tof_{module['name']}": [] for module in tof_modules}
    collect_camera_stamps = False

    def observe(message, key):
        state[key] = message
        if collect_camera_stamps and key in camera_stamps:
            stamp = message.header.stamp
            timestamp = stamp.sec + stamp.nanosec / 1e9
            history = camera_stamps[key]
            if not history or timestamp > history[-1][0]:
                history.append((timestamp, time.monotonic()))
        if key in tof_stamps:
            stamp = message.header.stamp
            timestamp = stamp.sec + stamp.nanosec / 1e9
            history = tof_stamps[key]
            if not history or timestamp > history[-1]:
                history.append(timestamp)

    subscriptions = []
    pointcloud_subscription = None
    for topic, message_type, key in (
        ("/odom", Odometry, "odom"), ("/camera/color/image_raw", Image, "rgb"),
        ("/camera/depth/image_rect_raw", Image, "depth"), ("/camera/imu", Imu, "imu"),
        ("/camera/color/camera_info", CameraInfo, "rgb_info"),
        ("/camera/depth/camera_info", CameraInfo, "depth_info"),
        ("/camera/depth/color/points", PointCloud2, "points"),
        ("/wheel_encoder_ticks", Int64MultiArray, "ticks"), ("/sim/cmd_vel", Twist, "command"),
    ):
        # 영상은 브리지와 같은 reliable 정책으로 계측하고, 작은 센서 메시지는
        # 일반 센서 QoS를 사용합니다. 구독자의 best-effort 누락을 혼동하지 않습니다.
        qos = QoSProfile(depth=5) if message_type is Image else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, key=key: observe(message, key), qos))
        if key == "points":
            pointcloud_subscription = subscriptions[-1]
    tof_keys = []
    for module in tof_modules:
        key = f"tof_{module['name']}"
        tof_keys.append(key)
        subscriptions.append(node.create_subscription(
            PointCloud2, f"{module['topic']}/points",
            lambda message, key=key: observe(message, key), qos_profile_sensor_data,
        ))
    publisher = node.create_publisher(AckermannDriveStamped, "/drive", 10)
    suffix_parts = []
    if args.d435i_profile != "configured":
        suffix_parts.append(profile_name)
    if args.tof_profile != "configured":
        suffix_parts.append(tof_profile_name)
    suffix = "" if not suffix_parts else "_" + "_".join(suffix_parts)
    log_path = output / f"{args.track}{suffix}_smoke.log"
    report_path = output / f"{args.track}{suffix}_smoke.json"
    report = {
        "track": args.track, "passed": False, "full_lap_test": False,
        "d435i_profile": profile_name, "tof_profile": tof_profile_name,
        "fastdds_builtin_transports": os.environ["FASTDDS_BUILTIN_TRANSPORTS"],
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "vehicle_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
    }
    log_stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "launch", "--noninteractive", "arena_bringup", "simulation.launch.py", "headless:=true", f"track:={args.track}",
         f"d435i_profile:={args.d435i_profile}", f"tof_profile:={args.tof_profile}"],
        cwd=repo, stdin=subprocess.DEVNULL, stdout=log_stream, stderr=subprocess.STDOUT, start_new_session=True,
    )

    def wait_for(predicate, timeout=45):
        deadline = time.monotonic() + timeout
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError(f"시뮬레이터 조기 종료: {process.returncode}; {log_path}")
            if time.monotonic() >= deadline:
                raise RuntimeError(f"관측 시간 초과; 수신 항목={sorted(state)}; {log_path}")
            rclpy.spin_once(node, timeout_sec=.05)

    def sim_time():
        stamp = state["odom"].header.stamp
        return stamp.sec + stamp.nanosec / 1e9

    def pose():
        p = state["odom"].pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (p.position.x, p.position.y, yaw)

    def drive(speed, steering, duration):
        start = sim_time()
        deadline = time.monotonic() + 45
        next_publish = 0.0
        while sim_time() - start < duration:
            now = time.monotonic()
            if process.poll() is not None or now > deadline:
                raise RuntimeError("주행 구간에서 시뮬레이션 진행이 멈췄습니다.")
            if now >= next_publish:
                message = AckermannDriveStamped()
                message.drive.speed = speed
                message.drive.steering_angle = steering
                publisher.publish(message)
                next_publish = now + .05
            # 명령 발행만 제한하고 영상·IMU 수신 처리는 계속합니다. 모든 콜백
            # 뒤에 sleep을 넣으면 reliable 영상 큐가 주행 중 밀릴 수 있습니다.
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        print(f"[{args.track}] 차량·센서 준비 대기", flush=True)
        required = ("odom", "rgb", "depth", "rgb_info", "depth_info", "points", "imu", "ticks", *tof_keys)
        wait_for(lambda: all(key in state for key in required), timeout=60)
        for key in tof_keys:
            tof_stamps[key].clear()  # 준비 단계의 짧은 첫 프레임 간격을 제외합니다.
        wait_for(lambda: all(len(tof_stamps[key]) >= 2 and tof_stamps[key][-1] - tof_stamps[key][0] >= 1.0
                             for key in tof_keys), timeout=30)
        expected_tof_size = [int(tof_profile["horizontal_zones"]), int(tof_profile["vertical_zones"])]
        tof_report = {}
        for module, key in zip(tof_modules, tof_keys):
            message = state[key]
            actual_size = [message.width, message.height]
            fields = [field.name for field in message.fields]
            if actual_size != expected_tof_size:
                raise AssertionError(f"{module['name']} ToF 점군 크기: {actual_size}; 목표: {expected_tof_size}")
            if message.header.frame_id != module["frame_id"]:
                raise AssertionError(
                    f"{module['name']} ToF 좌표계: {message.header.frame_id}; 목표: {module['frame_id']}"
                )
            if fields[:3] != ["x", "y", "z"]:
                raise AssertionError(f"{module['name']} ToF 점군 필드: {fields}")
            stamps = tof_stamps[key]
            observed_rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
            if not math.isclose(observed_rate, float(tof_profile["update_rate_hz"]), rel_tol=.05):
                raise AssertionError(
                    f"{module['name']} ToF 시뮬레이션 시간 기준 주기: {observed_rate}; "
                    f"목표: {tof_profile['update_rate_hz']}"
                )
            tof_report[module["name"]] = {
                "topic": f"{module['topic']}/points", "size": actual_size,
                "frame_id": message.header.frame_id, "fields": fields,
                "observed_sim_rate_hz": observed_rate, "samples": len(stamps),
            }
        report["tof_validation"] = {
            "enabled": bool(tof_config["enabled"]),
            "profile": tof_profile_name,
            "target_rate_hz": float(tof_profile["update_rate_hz"]),
            "modules": tof_report,
        }
        # 포인트 클라우드의 존재·형식을 먼저 확인한 뒤 기본 영상 파이프라인의
        # 처리율을 계측합니다. 불필요한 대용량 스트림은 지연 구독으로 중단됩니다.
        node.destroy_subscription(pointcloud_subscription)
        print(f"[{args.track}] {profile_name} 카메라 프레임률·내부 파라미터 검사", flush=True)
        warmup_started = sim_time()
        wait_for(lambda: sim_time() - warmup_started >= .5)
        collect_camera_stamps = True
        wait_for(lambda: len(camera_stamps["rgb"]) >= profile["color_rate_hz"] * 1.5
                 and len(camera_stamps["depth"]) >= profile["depth_rate_hz"] * 1.5, timeout=60)
        collect_camera_stamps = False
        camera_report = {}
        report["camera_validation"] = camera_report
        for key, stream_key, rate_key in (("rgb", "color", "color_rate_hz"), ("depth", "depth", "depth_rate_hz")):
            image_message = state[key]
            info = state[f"{key}_info"]
            stream = camera_config[stream_key]
            stamps = camera_stamps[key]
            sim_span = stamps[-1][0] - stamps[0][0]
            wall_span = stamps[-1][1] - stamps[0][1]
            observed_rate = (len(stamps) - 1) / sim_span
            expected_fx = (stream["width_px"] / 2) / math.tan(math.radians(stream["horizontal_fov_deg"]) / 2)
            expected_fy = (stream["height_px"] / 2) / math.tan(math.radians(stream["vertical_fov_deg"]) / 2)
            camera_report[key] = {
                "size": [image_message.width, image_message.height], "encoding": image_message.encoding,
                "frame_id": image_message.header.frame_id, "camera_info_k": list(info.k),
                "target_rate_hz": profile[rate_key], "observed_sim_rate_hz": observed_rate,
                "observed_wall_rate_hz": (len(stamps) - 1) / wall_span,
                "samples": len(stamps), "span_sim_s": sim_span, "real_time_factor": sim_span / wall_span,
                "interval_counts_ms": dict(sorted(Counter(round((b[0] - a[0]) * 1000, 3)
                                                          for a, b in zip(stamps, stamps[1:])).items())),
            }
            info_stamps = camera_stamps[f"{key}_info"]
            if len(info_stamps) > 1:
                camera_report[key]["info_observed_sim_rate_hz"] = (
                    (len(info_stamps) - 1) / (info_stamps[-1][0] - info_stamps[0][0])
                )
            if [image_message.width, image_message.height] != [stream["width_px"], stream["height_px"]]:
                raise AssertionError(f"{key} 영상 해상도가 설정과 다릅니다.")
            if [info.width, info.height] != [image_message.width, image_message.height]:
                raise AssertionError(f"{key} CameraInfo 해상도가 영상과 다릅니다.")
            if not math.isclose(observed_rate, profile[rate_key], rel_tol=.05):
                raise AssertionError(f"{key} 시뮬레이션 시간 기준 프레임률: {observed_rate}; 목표: {profile[rate_key]}")
            expected_frame = f"camera_{stream_key}_optical_frame"
            if image_message.header.frame_id != expected_frame or info.header.frame_id != expected_frame:
                raise AssertionError(f"{key} 영상·CameraInfo의 광학 좌표계가 설정과 다릅니다.")
            for actual, expected in ((info.k[0], expected_fx), (info.k[4], expected_fy),
                                     (info.k[2], stream["width_px"] / 2), (info.k[5], stream["height_px"] / 2)):
                if not math.isclose(actual, expected, abs_tol=1e-4):
                    raise AssertionError(f"{key} CameraInfo 내부 파라미터: {list(info.k)}")
        rgb_times = {stamp[0] for stamp in camera_stamps["rgb"]}
        depth_times = {stamp[0] for stamp in camera_stamps["depth"]}
        matching_fraction = len(rgb_times & depth_times) / min(len(rgb_times), len(depth_times))
        report["same_stamp_pair_fraction"] = matching_fraction
        if profile["hardware_sync_compatible"] and matching_fraction < .90:
            raise AssertionError("동일 프레임률 프로필에서 RGB·깊이 시각이 충분히 일치하지 않습니다.")
        if state["rgb"].encoding != "rgb8" or state["depth"].encoding != "32FC1":
            raise AssertionError("시뮬레이션 영상 인코딩이 예상과 다릅니다.")
        depth_values = array("f", state["depth"].data.tobytes())
        if bool(state["depth"].is_bigendian) != (sys.byteorder == "big"):
            depth_values.byteswap()
        valid_depth = [value for value in depth_values if math.isfinite(value) and value > 0]
        near = camera_config["depth"]["minimum_depth_m"]
        far = camera_config["depth"]["simulation_far_clip_m"]
        if not valid_depth or min(valid_depth) < near - 1e-4 or max(valid_depth) > far + 1e-4:
            raise AssertionError("깊이 영상의 유효 거리 범위가 설정과 다릅니다.")
        report["depth_valid_range_observed_m"] = [min(valid_depth), max(valid_depth)]
        report["pointcloud_fields"] = [field.name for field in state["points"].fields]
        report["pointcloud_size"] = [state["points"].width, state["points"].height]
        initial = pose()
        initial_ticks = list(state["ticks"].data)
        print(f"[{args.track}] 직진 검사", flush=True)
        drive(.20, 0., 1.2)
        after_straight = pose()
        print(f"[{args.track}] 조향 검사", flush=True)
        drive(.20, .15, 1.0)
        after_turn = pose()
        end_ticks = list(state["ticks"].data)
        print(f"[{args.track}] 명령 중단 정지 검사", flush=True)
        stopped_at = sim_time()
        wait_for(lambda: sim_time() - stopped_at >= 2.0
                 and abs(state["odom"].twist.twist.linear.x) < .01)
        forward_distance = math.dist(initial[:2], after_straight[:2])
        yaw_change = math.atan2(math.sin(after_turn[2] - after_straight[2]),
                                math.cos(after_turn[2] - after_straight[2]))
        if forward_distance < .10 or abs(yaw_change) < .01:
            raise AssertionError(f"이동 또는 회전이 부족합니다: {forward_distance}, {yaw_change}")
        deltas = [end - start for start, end in zip(initial_ticks, end_ticks)]
        if len(deltas) != 2 or not all(abs(delta) > 10 for delta in deltas) or deltas[0] == deltas[1]:
            raise AssertionError(f"좌우 엔코더 변화가 예상과 다릅니다: {deltas}")
        if "command" not in state or state["command"].linear.x != 0 or state["command"].angular.z != 0:
            raise AssertionError("명령 수신 중단 감시 기능이 정지 명령을 보내지 않았습니다.")
        if abs(state["odom"].twist.twist.linear.x) >= .01:
            raise AssertionError("정지 대기 후에도 차량 속도가 큽니다.")
        report.update({
            "passed": True, "straight_distance_m": forward_distance, "turn_yaw_change_rad": yaw_change,
            "encoder_tick_deltas": deltas, "rgb_size": [state["rgb"].width, state["rgb"].height],
            "depth_encoding": state["depth"].encoding, "imu_received": True,
            "watchdog_stop_observed": True, "stopped_speed_mps": state["odom"].twist.twist.linear.x,
        })
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        # 대용량 영상 수신자를 먼저 해제한 뒤 송신 측을 종료합니다.
        node.destroy_node()
        rclpy.shutdown()
        # SIGINT는 ROS launch에만 보냅니다. 전체 그룹에 보내면 launch의 전달과
        # 겹쳐 자식이 정리 도중 두 번째 KeyboardInterrupt를 받을 수 있습니다.
        # 응답하지 않을 때만 이번 검사에서 만든 그룹 전체에 종료 신호를 보냅니다.
        for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            if process.poll() is not None:
                break
            try:
                if sig == signal.SIGINT:
                    process.send_signal(sig)
                else:
                    os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
        report["launch_exit_code"] = process.poll()
        log_stream.close()
        log_text = log_path.read_text(encoding="utf-8")
        errors = [line for line in log_text.splitlines()
                  if "Traceback" in line or "process has died" in line or "[ERROR]" in line]
        report["shutdown_clean"] = not errors and process.returncode == 0
        if errors or not report["shutdown_clean"]:
            report["passed"] = False
            report["process_errors"] = errors
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] and report["launch_exit_code"] is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
