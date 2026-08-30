#!/usr/bin/env python3
"""선택한 월드에서 짧은 직진·회전·센서·명령 중단 정지를 검증합니다. 완주 시험은 아닙니다."""

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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", choices=("original", "experimental"), default="experimental")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    output = repo / "artifacts/tests"
    output.mkdir(parents=True, exist_ok=True)
    # 다른 사용자의 시뮬레이션과 ROS/Gazebo 명령이 섞이지 않게 격리합니다.
    os.environ["ROS_DOMAIN_ID"] = str(70 + os.getpid() % 100)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["GZ_PARTITION"] = f"it_arena_smoke_{os.getpid()}"

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import Int64MultiArray

    rclpy.init()
    node = rclpy.create_node("arena_smoke_test")
    state = {}
    subscriptions = []
    for topic, message_type, key in (
        ("/odom", Odometry, "odom"), ("/camera/color/image_raw", Image, "rgb"),
        ("/camera/depth/image_rect_raw", Image, "depth"), ("/camera/imu", Imu, "imu"),
        ("/wheel_encoder_ticks", Int64MultiArray, "ticks"), ("/sim/cmd_vel", Twist, "command"),
    ):
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, key=key: state.__setitem__(key, message), qos_profile_sensor_data))
    publisher = node.create_publisher(AckermannDriveStamped, "/drive", 10)
    log_path = output / f"{args.track}_smoke.log"
    report_path = output / f"{args.track}_smoke.json"
    report = {
        "track": args.track, "passed": False, "full_lap_test": False,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "vehicle_config_sha256": hashlib.sha256((repo / "src/arena_description/config/vehicle.yaml").read_bytes()).hexdigest(),
    }
    log_stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "launch", "arena_bringup", "simulation.launch.py", "headless:=true", f"track:={args.track}"],
        cwd=repo, stdout=log_stream, stderr=subprocess.STDOUT, start_new_session=True,
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
        while sim_time() - start < duration:
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("주행 구간에서 시뮬레이션 진행이 멈췄습니다.")
            message = AckermannDriveStamped()
            message.drive.speed = speed
            message.drive.steering_angle = steering
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.03)
            # 고속 토픽 수신이 많아도 명령을 과도하게 발행하지 않습니다.
            time.sleep(.02)

    try:
        print(f"[{args.track}] 차량·센서 준비 대기", flush=True)
        wait_for(lambda: all(key in state for key in ("odom", "rgb", "depth", "imu", "ticks")), timeout=60)
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
        node.destroy_node()
        rclpy.shutdown()
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
