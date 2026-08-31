#!/usr/bin/env python3
"""독립 시험장에서 단일 모터·차동·접지·ToF 제동을 계측합니다.

정답 위치/속도는 이 시험기의 평가에만 사용합니다. 자율주행 입력이 아닙니다.
위험 시나리오에서 미끄러짐/전복 관측은 '안전 주행 통과'를 뜻하지 않습니다.
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
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
CASES = {
    "low_speed_curve": {"speed": .7, "steer": .25, "duration": 6.0},
    "straight_20kmh": {"speed": 20 / 3.6, "steer": 0, "duration": 6.0},
    "corner_5kmh": {"speed": 5 / 3.6, "steer": .37, "duration": 6.0},
    "corner_8kmh": {"speed": 8 / 3.6, "steer": .37, "duration": 6.0},
    "split_grip_open": {"speed": 2., "steer": 0, "duration": 5., "split": True},
    "split_grip_lsd": {"speed": 2., "steer": 0, "duration": 5., "split": True, "differential": "viscous_lsd"},
    "high_cg_high_grip": {"speed": 8 / 3.6, "steer": .37, "duration": 6., "stress": True},
    "high_cg_lsd": {"speed": 3.0, "steer": .37, "duration": 7., "stress": True, "differential": "viscous_lsd"},
    "curb_trip": {"speed": 20 / 3.6, "steer": 0, "duration": 5., "curb": True},
    "bump_20kmh": {"speed": 20 / 3.6, "steer": 0, "duration": 5., "bump": True},
    "tof_stop": {"speed": 1.4, "steer": 0, "duration": 9., "tof": True, "obstacle_x": 4.0},
    "tof_reverse_stop": {"speed": -.7, "steer": 0, "duration": 10., "tof": True, "obstacle_x": -4.0},
    "tof_unsafe_speed": {"speed": 20 / 3.6, "steer": 0, "duration": 6., "tof": True,
                         "obstacle_x": 12., "safety": False},
}


def trial_world(output, case):
    root = ET.parse(REPO / "src/arena_gazebo/worlds/vehicle_dynamics_lab/world.sdf")
    if "obstacle_x" in case:
        front = case["obstacle_x"]
        center = front + math.copysign(.10, front)
        model = ET.SubElement(root.getroot().find("world"), "model", name="low_parked_target")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{center} 0 .025 0 0 0"
        link = ET.SubElement(model, "link", name="body")
        for kind in ("visual", "collision"):
            component = ET.SubElement(link, kind, name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(component, "geometry"), "box"), "size").text = ".20 .15 .05"
    if case.get("curb") or case.get("bump"):
        obstacle = ET.SubElement(root.getroot().find("world"), "model", name="contact_stress_fixture")
        ET.SubElement(obstacle, "static").text = "true"
        link = ET.SubElement(obstacle, "link", name="fixture")
        if case.get("curb"):
            # 경기 시설이 아닌 한쪽 바퀴/차체 걸림용 독립 스트레스 표적.
            segments = [(10.1, .09, .020, 0., .20, .08, .04)]
        else:
            # 기존 실험 방지턱과 같은 20 cm 길이/1 cm 높이 코사인 융기.
            segments = []
            for i in range(40):
                x0, x1 = i * .005, (i + 1) * .005
                z0, z1 = [.005 * (1 - math.cos(2 * math.pi * x / .2)) for x in (x0, x1)]
                pitch = -math.atan2(z1 - z0, x1 - x0)
                segments.append((10 + (x0 + x1) / 2, 0, (z0 + z1) / 2 - .001 * math.cos(pitch),
                                 pitch, math.hypot(x1 - x0, z1 - z0) + .00001, .45, .002))
        for index, (x, y, z, pitch, length, width, height) in enumerate(segments):
            for kind in ("collision", "visual"):
                shape = ET.SubElement(link, kind, name=f"{kind}_{index}")
                ET.SubElement(shape, "pose").text = f"{x} {y} {z} 0 {pitch} 0"
                ET.SubElement(ET.SubElement(ET.SubElement(shape, "geometry"), "box"), "size").text = f"{length} {width} {height}"
    path = output / "trial_world.sdf"
    root.write(path, encoding="utf-8", xml_declaration=True)
    return path


def summarize(trace, case):
    active = [s for s in trace if 0.0 <= s["elapsed_s"] < case["duration"]]
    final = [s for s in trace if s["elapsed_s"] >= case["duration"] + 2.5]
    if not active or not final:
        raise RuntimeError("시험 구간 표본이 부족합니다.")
    metric = lambda field, rows=active: max(abs(s[field]) for s in rows)
    peak = metric("truth_longitudinal_mps")
    longitudinal_slip = [s["wheel_surface_speed_mps"] - s["truth_longitudinal_mps"] for s in active]
    result = {
        "target_speed_mps": case["speed"], "peak_ground_speed_mps": peak,
        "peak_ground_speed_kmh": peak * 3.6,
        "ground_speed_note": "ground_speed fields are body longitudinal speed; planar_speed includes lateral motion",
        "peak_planar_speed_mps": max(math.hypot(s["truth_longitudinal_mps"], s["truth_lateral_mps"]) for s in active),
        "peak_lateral_speed_mps": metric("truth_lateral_mps"),
        "peak_roll_deg": math.degrees(metric("truth_roll_rad")),
        "peak_pitch_deg": math.degrees(metric("truth_pitch_rad")),
        "peak_chassis_height_m": max(s["truth_z_m"] for s in active),
        "peak_longitudinal_slip_speed_mps": max(map(abs, longitudinal_slip)),
        "peak_wheel_speed_difference_rad_s": max(abs(s["left_speed_rad_s"] - s["right_speed_rad_s"]) for s in active),
        "peak_torque_difference_nm": max(abs(s["left_torque_nm"] - s["right_torque_nm"]) for s in active),
        "motor_average_relation_error_rad_s": max(abs(s["motor_speed_rad_s"] - 8 *
            (s["left_speed_rad_s"] + s["right_speed_rad_s"]) / 2) for s in active),
        "min_loss_power_w": min(s["loss_power_w"] for s in active),
        "median_ground_speed_last_active_second_mps": float(np.median([s["truth_longitudinal_mps"] for s in active
                                                             if s["elapsed_s"] >= case["duration"] - 1])),
        "final_ground_speed_mps": metric("truth_longitudinal_mps", final),
        "final_lateral_speed_mps": metric("truth_lateral_mps", final),
        "final_planar_speed_mps": max(math.hypot(s["truth_longitudinal_mps"], s["truth_lateral_mps"]) for s in final),
        "final_wheel_speed_mps": metric("wheel_surface_speed_mps", final),
        "encoder_messages_seen": any(s.get("encoder_received") for s in active),
        "safety_states": sorted({s["safety"]["state"] for s in trace if "safety" in s}),
        "watchdog_stop_seen": any(s["watchdog_stop"] for s in final),
        "rolled_over": any(abs(s["truth_roll_rad"]) > math.pi / 3 or
                           abs(s["truth_pitch_rad"]) > math.pi / 3 for s in active),
    }
    result["stopped"] = result["final_planar_speed_mps"] < .05 and result["final_wheel_speed_mps"] < .05
    flat_turns = [s for s in active if abs(s["truth_roll_rad"]) < .1 and abs(s["truth_pitch_rad"]) < .1
                  and abs(s["truth_yaw_rate_rad_s"]) > .2 and abs(s["truth_longitudinal_mps"]) > .3]
    if flat_turns:
        result["median_actual_turn_radius_m"] = float(np.median([
            abs(s["truth_longitudinal_mps"] / s["truth_yaw_rate_rad_s"]) for s in flat_turns]))
        result["peak_rear_axle_lateral_slip_proxy_mps"] = max(abs(s["truth_lateral_mps"] -
            s["truth_yaw_rate_rad_s"] * .145 / 2) for s in flat_turns)
        # 평지 근사. 차체 축간 중앙의 Y 속도 자체는 정상 선회에서도 0이 아닙니다.
        result["rear_lateral_proxy_note"] = "flat attitude only; chassis Vy minus yaw_rate*wheelbase/2"
    if "obstacle_x" in case:
        sign = 1 if case["obstacle_x"] > 0 else -1
        # 이번 표적은 축 정렬 정적 상자. yaw에 따른 외형 팽창과 Y 겹침을 함께 확인합니다.
        clearances = []
        for s in trace:
            yaw = s["truth_yaw_rad"]
            ex = .10 * abs(math.cos(yaw)) + .075 * abs(math.sin(yaw))
            ey = .10 * abs(math.sin(yaw)) + .075 * abs(math.cos(yaw))
            if abs(s["truth_y_m"]) < ey + .075:
                clearances.append(abs(case["obstacle_x"]) - sign * s["truth_x_m"] - ex)
        result["minimum_target_clearance_m"] = min(clearances) if clearances else None
        result["target_contact_or_crossing"] = bool(clearances and min(clearances) <= .002)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=list(CASES), default="low_speed_curve")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--physics-step", type=float, default=.001)
    args = parser.parse_args()
    case = CASES[args.case]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = (args.output or REPO / "artifacts/tests/vehicle_dynamics" / f"{stamp}_{args.case}").resolve()
    if not output.is_relative_to(REPO / "artifacts/tests"):
        raise ValueError("실행 자료는 artifacts/tests 안에 보관합니다.")
    output.mkdir(parents=True, exist_ok=False)
    if not math.isfinite(args.physics_step) or not .0001 <= args.physics_step <= .002:
        raise ValueError("물리 시간 간격은 0.1~2 ms 범위여야 합니다.")
    os.environ.update(ROS_DOMAIN_ID=str(80 + os.getpid() % 120), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                      ROS_STATIC_PEERS="", GZ_PARTITION=f"arena_dynamics_{os.getpid()}")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    config = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text())
    tire = config["vehicle"]["drivetrain"]["tire_contact"]
    if case.get("split"):
        tire["rear_left_friction_scale"] = .04
    if case.get("stress"):
        config["vehicle"]["body"]["center_of_mass_z_offset_m"] = .055
        tire.update(friction_lateral=1.6, friction_longitudinal=1.6,
                    slip_compliance_lateral=.005, slip_compliance_longitudinal=.005)
    config_path = output / "trial_vehicle.yaml"
    config_path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    world = trial_world(output, case)
    root = ET.parse(world)
    root.getroot().find("world/physics/max_step_size").text = str(args.physics_step)
    root.write(world, encoding="utf-8", xml_declaration=True)
    subprocess.run(["gz", "sdf", "-k", str(world)], check=True, capture_output=True, text=True)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from ackermann_msgs.msg import AckermannDriveStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("vehicle_dynamics_validator")
    current, trace = {}, []
    interrupted = False
    def on_signal(*_):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    def receive(message, key):
        current[key] = json.loads(message.data)
    subscriptions = [
        node.create_subscription(String, "/sim/drivetrain", lambda m: receive(m, "dynamics"), 20),
        node.create_subscription(String, "/safety/status", lambda m: receive(m, "safety"), 20),
        node.create_subscription(JointState, "/wheel_states", lambda m: current.update(encoder=m), qos_profile_sensor_data),
    ]
    publisher = node.create_publisher(AckermannDriveStamped, "/drive", 10)
    report = {"case": args.case, "started_at_utc": stamp, "passed": False, "safe_racing_verified": False,
              "physics_step_s": args.physics_step, "case_configuration": case,
              "input_sha256": {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in [REPO / "src/arena_gazebo/src/single_motor_drive.cpp",
                             REPO / "src/arena_gazebo/include/arena_gazebo/drivetrain.hpp",
                             REPO / "src/arena_description/models/arena_car/model.sdf.xacro",
                             REPO / "src/arena_description/config/vehicle.yaml",
                             REPO / "src/arena_bringup/launch/simulation.launch.py",
                             REPO / "src/arena_bringup/config/tof_safety.yaml",
                             REPO / "src/arena_vehicle_interface/arena_vehicle_interface/ackermann_to_twist.py",
                             REPO / "src/arena_vehicle_interface/arena_vehicle_interface/sim_wheel_encoder.py",
                             REPO / "src/arena_autonomy/arena_autonomy/tof_safety.py",
                             REPO / "src/arena_autonomy/arena_autonomy/tof_safety_core.py",
                             Path(__file__).resolve(), config_path, world]}}
    process = None
    log_path = output / "simulation.log"
    log = log_path.open("w", encoding="utf-8")
    stream = (output / "trace.jsonl").open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(["ros2", "launch", "--noninteractive", "arena_bringup", "simulation.launch.py",
            "headless:=true", f"vehicle_config:={config_path}", f"world_override:={world}",
            f"differential_profile:={case.get('differential', 'ideal_open')}",
            f"render_sensors:={str(bool(case.get('tof'))).lower()}",
            f"tof_safety:={str(bool(case.get('tof')) and case.get('safety', True)).lower()}",
            "depth_camera:=false", "d435i_profile:=low_load_30"], cwd=REPO, stdin=subprocess.DEVNULL,
            stdout=log, stderr=log, start_new_session=True)
        began = time.monotonic()
        start = None
        last_sample = last_command = -math.inf
        last_progress = 0
        while True:
            rclpy.spin_once(node, timeout_sec=.002)
            if interrupted:
                raise RuntimeError("사용자 중단 요청")
            if process.poll() is not None:
                raise RuntimeError(f"시뮬레이터 조기 종료: {process.returncode}")
            if time.monotonic() - began > 220:
                raise RuntimeError("실제 시간 제한 초과")
            if "dynamics" not in current:
                continue
            state = current["dynamics"]
            now = state["sim_time_s"]
            if start is None:
                start = now + 1.0
            elapsed = now - start
            if elapsed > case["duration"] + 3.2:
                break
            if now - last_command >= .019 and elapsed < case["duration"]:
                command = AckermannDriveStamped()
                command.drive.speed = float(case["speed"] if elapsed >= 0 else 0)
                command.drive.steering_angle = float(case["steer"] if elapsed >= 2 else 0)
                publisher.publish(command)
                last_command = now
            # 요청 발행을 중단해 adapter/ToF/구동 플러그인의 감시 경로를 시험합니다.
            if now > last_sample:
                row = {**state, "elapsed_s": elapsed, "encoder_received": "encoder" in current}
                if "safety" in current:
                    row["safety"] = current["safety"]
                trace.append(row)
                stream.write(json.dumps(row, allow_nan=False) + "\n")
                last_sample = now
            if time.monotonic() - last_progress > 10:
                print(f"{args.case}: t={elapsed:.2f}s, ground={state['truth_longitudinal_mps']:.3f}m/s, "
                      f"roll={math.degrees(state['truth_roll_rad']):.1f}deg, "
                      f"safety={current.get('safety', {}).get('state', 'OFF')}", flush=True)
                last_progress = time.monotonic()
                stream.flush()
        report.update(summarize(trace, case))
        basic = report["encoder_messages_seen"] and report["motor_average_relation_error_rad_s"] < 1e-6
        if args.case == "low_speed_curve":
            report["scenario_expectation"] = "전진·차동·조향·정지"
            report["passed"] = basic and report["peak_ground_speed_mps"] > .5 and report["stopped"] and report["peak_wheel_speed_difference_rad_s"] > 1
        elif args.case == "straight_20kmh":
            report["scenario_expectation"] = "평지 시험장에서 목표의 95% 이상 도달·정지"
            report["passed"] = basic and report["peak_ground_speed_mps"] > 20 / 3.6 * .95 and report["stopped"]
        elif args.case.startswith("tof_") and case.get("safety", True):
            report["scenario_expectation"] = "저상 표적 앞에서 실제 이동 후 비접촉 정지"
            report["passed"] = (basic and report["peak_ground_speed_mps"] > .3 and report["stopped"] and
                                "OBSTACLE_STOP" in report["safety_states"] and not report["target_contact_or_crossing"])
        else:
            report["scenario_expectation"] = "물성 민감도 계측. 위험 현상도 결과로 보존하며 안전 주행 통과 아님"
            report["passed"] = basic
    except Exception as error:
        report["error"] = str(error)
    finally:
        publisher.publish(AckermannDriveStamped())
        node.destroy_node()
        rclpy.shutdown()
        escalated = False
        if process is not None:
            for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                if process.poll() is not None:
                    break
                try:
                    if sig == signal.SIGINT:
                        process.send_signal(sig)
                    else:
                        escalated = True
                        os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    continue
        stream.close()
        log.close()
        errors = [line for line in log_path.read_text(errors="replace").splitlines()
                  if any(token in line for token in ("[ERROR]", "[Err]", "Traceback", "Segmentation fault"))]
        report.update(shutdown_clean=process is not None and process.returncode == 0 and not escalated and not errors,
                      process_exit_code=process.returncode if process else None, process_errors=errors,
                      trace_samples=len(trace), elapsed_wall_s=time.monotonic() - began if process else 0)
        report["passed"] = report["passed"] and report["shutdown_clean"]
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps({key: value for key, value in report.items() if key != "input_sha256"}, ensure_ascii=False, indent=2))
        print(f"REPORT: {output / 'report.json'}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
