#!/usr/bin/env python3
"""공식 원위치 마커의 재질만 단계별로 바꾸고 실제 Gazebo RGB를 저장합니다.

입력 사본의 링크/접촉/시설 위치를 유지하며 진단 카메라 네 개만 추가합니다.
기본값은 렌더링 원인 분리를 위해 Physics를 제외하며, --physics로 물리를
함께 켠 최종 상태도 확인할 수 있습니다. 실차/주행 중 인식 시험은 아닙니다.
"""

from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from audit_official_world_load import REPO, descendants_alive, sha256, write_json


def stop_test(process: subprocess.Popen) -> dict:
    signals = []
    for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        if not descendants_alive(process.pid):
            break
        os.killpg(process.pid, sig)
        signals.append(sig.name)
        deadline = time.monotonic() + grace
        while descendants_alive(process.pid) and time.monotonic() < deadline:
            time.sleep(.1)
    process.wait(timeout=2)
    return {"return_code": process.returncode, "signals": signals,
            "remaining_test_pids": descendants_alive(process.pid)}


def make_world(base: Path, case: str, output: Path, fixture: str | None = None) -> dict:
    source_case = fixture or ("remove_script" if case == "path_bad" else "script_and_paths")
    source = base / "fixtures" / source_case / "output_final"
    world = output / "world.sdf"
    root = ET.parse(source / "world.sdf").getroot()
    model = root.find("world/model")
    assert model is not None
    original_model = ET.tostring(model)
    for visual in model.findall("link/visual"):
        if visual.attrib["name"].startswith("aruco_") and case == "diffuse_white":
            material = visual.find("material")
            assert material is not None and material.find("diffuse") is None
            ET.SubElement(material, "diffuse").text = "1 1 1 1"
    model_after_material = copy.deepcopy(model)
    if case == "diffuse_white":
        for material in model_after_material.findall("link/visual/material"):
            if material.find("pbr") is not None:
                material.remove(material.find("diffuse"))
    assert ET.tostring(model_after_material) == original_model
    camera_poses = {}
    world_element = root.find("world")
    for marker_id in (0, 20, 30, 45):
        marker = model.find(f"link[@name='aruco_{marker_id}']")
        x, y, z, roll, pitch, yaw = map(float, marker.findtext("pose").split())
        assert roll == 0 and pitch == 0
        # 마커 설계의 정면(+local X) 쪽에서 판 중심을 바라봅니다.
        distance = .35
        camera_yaw = math.atan2(-math.sin(yaw), -math.cos(yaw))
        camera_pose = [x + distance * math.cos(yaw), y + distance * math.sin(yaw), z, 0, 0, camera_yaw]
        camera_poses[str(marker_id)] = camera_pose
        camera = ET.SubElement(world_element, "model", name=f"audit_camera_{marker_id}")
        ET.SubElement(camera, "static").text = "true"
        ET.SubElement(camera, "pose").text = " ".join(map(str, camera_pose))
        link = ET.SubElement(camera, "link", name="link")
        sensor = ET.SubElement(link, "sensor", name="rgb", type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "2"
        ET.SubElement(sensor, "topic").text = f"/audit/marker_{marker_id}/rgb"
        config = ET.SubElement(sensor, "camera")
        ET.SubElement(config, "horizontal_fov").text = ".65"
        image = ET.SubElement(config, "image")
        ET.SubElement(image, "width").text = "640"
        ET.SubElement(image, "height").text = "480"
        ET.SubElement(image, "format").text = "R8G8B8"
        clip = ET.SubElement(config, "clip")
        ET.SubElement(clip, "near").text = ".01"
        ET.SubElement(clip, "far").text = "5"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(world, encoding="utf-8", xml_declaration=True)
    shutil.copytree(source / "aruco", output / "aruco")
    return {"source_case": source_case, "source_world_sha256": sha256((source / "world.sdf").read_bytes()),
            "render_world_sha256": sha256(world.read_bytes()), "camera_pose_xyz_rpy": camera_poses,
            "camera": {"width": 640, "height": 480, "horizontal_fov_rad": .65,
                       "distance_m": .35, "update_rate_hz": 2},
            "track_links": len(model.findall("link")),
            "source_model_unchanged_except_diffuse": True}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("path_bad", "path_fixed", "diffuse_white", "as_is"))
    parser.add_argument("--workdir", type=Path, default=REPO / "build/official_load_audit_20260831")
    parser.add_argument("--timeout", type=float, default=55)
    parser.add_argument("--fixture", choices=("raw", "remove_script", "script_and_paths", "merge_only", "merge_static", "ready"))
    parser.add_argument("--output-name")
    parser.add_argument("--physics", action="store_true")
    parser.add_argument("--min-frames", type=int, default=3)
    args = parser.parse_args()
    if args.case == "as_is" and args.fixture is None:
        parser.error("as_is에는 검사할 --fixture가 필요합니다.")
    output_name = args.output_name or args.case
    if not re.fullmatch(r"[A-Za-z0-9_-]+", output_name) or args.min_frames < 3:
        parser.error("output-name은 영문/숫자/_/-만, min-frames는 3 이상이어야 합니다.")
    base = args.workdir.resolve()
    if not base.is_relative_to((REPO / "build").resolve()):
        parser.error("작업 폴더는 저장소 build 아래여야 합니다.")
    output = base / "render" / output_name
    output.mkdir(parents=True, exist_ok=False)
    report = make_world(base, args.case, output, args.fixture)
    report.update({"case": args.case, "started_at": datetime.now(timezone.utc).isoformat(),
                   "physics_enabled": args.physics, "requested_frames": args.min_frames,
                   "scope": "원본 위치의 정지 마커와 별도 정면 진단 카메라. 실차/주행 검증 아님."})
    config = output / "server.config"
    physics = ('<plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" '
               'name="gz::sim::systems::Physics"/>' if args.physics else '')
    config.write_text("<server_config><plugins>" + physics + """
      <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin entity_name="*" entity_type="world" filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    </plugins></server_config>""", encoding="utf-8")
    os.environ["GZ_PARTITION"] = f"arena_marker_audit_{os.getpid()}"
    os.environ["GZ_SIM_SERVER_CONFIG_PATH"] = str(config)
    os.environ["ROS_DOMAIN_ID"] = str(180 + os.getpid() % 20)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node("official_material_audit")
    frames = {}
    counts = {i: 0 for i in (0, 20, 30, 45)}
    first_stamps = {}
    subscriptions = []
    def receive(message, marker_id):
        if message.encoding != "rgb8":
            raise ValueError(message.encoding)
        frames[marker_id] = message
        counts[marker_id] += 1
        first_stamps.setdefault(marker_id, message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
    for marker_id in counts:
        subscriptions.append(node.create_subscription(
            Image, f"/audit/marker_{marker_id}/rgb",
            lambda message, marker_id=marker_id: receive(message, marker_id), QoSProfile(depth=2)))
    server_command = ["stdbuf", "-oL", "-eL", "gz", "sim", "-s", "-r", "-v", "4", "world.sdf"]
    bridge_command = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *[
        f"/audit/marker_{i}/rgb@sensor_msgs/msg/Image[gz.msgs.Image" for i in counts]]
    report["commands"] = {"server": server_command, "bridge": bridge_command}
    processes = []
    logs = []
    try:
        for name, command in (("server", server_command), ("bridge", bridge_command)):
            log = (output / f"{name}.log").open("wb")
            logs.append(log)
            processes.append(subprocess.Popen(command, cwd=output, stdin=subprocess.DEVNULL,
                                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + args.timeout
        while min(counts.values()) < args.min_frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        report["received_counts"] = counts
        report["frames"] = {}
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker_id, message in frames.items():
            rgb = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
            name = f"marker_{marker_id}.png"
            assert cv2.imwrite(str(output / name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            _, mirrored_ids, _ = cv2.aruco.detectMarkers(np.fliplr(gray).copy(), dictionary)
            report["frames"][str(marker_id)] = {
                "file": name, "sha256": sha256((output / name).read_bytes()),
                "stamp_sec": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                "first_received_stamp_sec": first_stamps[marker_id],
                "detected_ids": [] if ids is None else ids.flatten().tolist(),
                "horizontally_flipped_diagnostic_ids": [] if mirrored_ids is None else mirrored_ids.flatten().tolist(),
                "center_200px_gray_min_max_mean": [int(gray[140:340,220:420].min()), int(gray[140:340,220:420].max()), float(gray[140:340,220:420].mean())],
            }
        report["capture_complete"] = len(frames) == 4 and min(counts.values()) >= args.min_frames
    finally:
        node.destroy_node()
        rclpy.shutdown()
        report["cleanup"] = [stop_test(process) for process in reversed(processes)]
        for log in logs:
            log.close()
        report["errors_warnings"] = {}
        for name in ("server", "bridge"):
            text = re.sub(r"\x1b\[[0-9;]*m", "", (output / f"{name}.log").read_text(encoding="utf-8", errors="replace"))
            report["errors_warnings"][name] = [line for line in text.splitlines() if any(term in line for term in ("[Err]", "[Wrn]", "Error", "Exception"))]
        write_json(output / "result.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["capture_complete"] and all(not r["remaining_test_pids"] for r in report["cleanup"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
