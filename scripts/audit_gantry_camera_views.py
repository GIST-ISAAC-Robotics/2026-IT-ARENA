#!/usr/bin/env python3
"""공식 마커를 그대로 두고 차량 카메라 상당 정지 시야·전체 사진을 검사합니다.

차량 동역학·차체 가림·노출·실물 성능은 시험하지 않습니다. 카메라 자세만
바꾸는 별도 진단이며 운영 차량과 마커 SDF를 수정하지 않습니다.
--marker-face sdf_cells는 진단 사본에만 팀의 도형 앞면을 추가합니다.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

from audit_official_marker_render import stop_test
from build_official_track import REPO, DESTINATION, sha256


def add_camera(world, name, pose, width, height, hfov, vfov=None):
    model = ET.SubElement(world, 'model', name='audit_' + name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ET.SubElement(model, 'link', name='link')
    sensor = ET.SubElement(link, 'sensor', name='rgb', type='camera')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '2'
    topic = '/gantry_audit/' + name
    ET.SubElement(sensor, 'topic').text = topic
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = str(hfov)
    frame = ET.SubElement(camera, 'image')
    for key, value in [('width', width), ('height', height), ('format', 'R8G8B8')]:
        ET.SubElement(frame, key).text = str(value)
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '.02'
    ET.SubElement(clip, 'far').text = '100' if vfov is None else '20'
    if vfov is not None:
        intrinsic = ET.SubElement(camera, 'intrinsics')
        for key, value in [('fx', width / 2 / math.tan(hfov / 2)),
                           ('fy', height / 2 / math.tan(vfov / 2)),
                           ('cx', width / 2), ('cy', height / 2), ('s', 0)]:
            ET.SubElement(intrinsic, key).text = str(value)
    return topic


def sample_route(csv, lap, s):
    s %= lap
    xs = np.append(csv['x_m'], csv['x_m'][0])
    ys = np.append(csv['y_m'], csv['y_m'][0])
    ss = np.append(csv['s_m'], lap)
    x, y = np.interp(s, ss, xs), np.interp(s, ss, ys)
    before, after = (s - .01) % lap, (s + .01) % lap
    yaw = math.atan2(np.interp(after, ss, ys) - np.interp(before, ss, ys),
                     np.interp(after, ss, xs) - np.interp(before, ss, xs))
    return float(x), float(y), yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DESTINATION)
    parser.add_argument('--name', required=True)
    parser.add_argument('--mode', choices=['route', 'overview'], default='route')
    parser.add_argument('--pitch-up-deg', type=float, default=0)
    parser.add_argument('--timeout', type=float, default=150)
    parser.add_argument('--marker-face', choices=['official', 'sdf_cells'], default='official')
    args = parser.parse_args()
    if not re.fullmatch('[a-zA-Z0-9_-]+', args.name):
        parser.error('name은 영문·숫자·밑줄·하이픈만 허용합니다.')
    output = REPO / 'artifacts/validation/2026-09-15/official_update' / args.name
    work = REPO / 'build/official_update_20260915/render' / args.name
    output.mkdir(parents=True, exist_ok=False)
    work.mkdir(parents=True, exist_ok=False)
    source = args.source.resolve()
    root = ET.parse(source / 'world.sdf').getroot()
    world = root.find('world')
    track = world.find("model[@name='it_arena_track_static']")
    original = ET.tostring(track)
    # 원본/실행본의 시스템 차이를 통제하며 모델 링크는 건드리지 않습니다.
    for plugin in list(world.findall('plugin')):
        world.remove(plugin)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name='gz::sim::systems::' + name)
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    scene = json.loads((source / 'scene.json').read_text(encoding='utf-8'))
    camera = yaml.safe_load((REPO / 'src/arena_description/config/vehicle.yaml').read_text())['vehicle']['sensors']['d435i']
    color = camera['color']
    views = {}
    if args.mode == 'route':
        route = np.genfromtxt(source / 'centerline.csv', delimiter=',', names=True)
        for marker in scene['aruco_markers']['markers']:
            for distance in (3.0, 2.0, 1.5, 1.2, 1.0, .75):
                x, y, yaw = sample_route(route, scene['track']['lap_length_m'], marker['s_m'] - distance)
                dx, dy, dz = camera['xyz_m']
                pose = [x + dx * math.cos(yaw) - dy * math.sin(yaw),
                        y + dx * math.sin(yaw) + dy * math.cos(yaw), dz + .003,
                        0, -math.radians(args.pitch_up_deg), yaw]
                name = f"id{marker['id']}_{round(distance * 100)}cm"
                topic = add_camera(world, name, pose, color['width_px'], color['height_px'],
                                   math.radians(color['horizontal_fov_deg']), math.radians(color['vertical_fov_deg']))
                views[name] = {'topic': topic, 'pose_xyz_rpy': pose, 'expected_id': marker['id'],
                               'route_distance_m': distance}
        for slot in scene['starting_grid']['slots']:
            yaw = slot['yaw_rad']
            dx, dy, dz = camera['xyz_m']
            pose = [slot['x'] + dx * math.cos(yaw) - dy * math.sin(yaw),
                    slot['y'] + dx * math.sin(yaw) + dy * math.cos(yaw), dz + .003,
                    0, -math.radians(args.pitch_up_deg), yaw]
            name = f"grid_{slot['index']}"
            topic = add_camera(world, name, pose, color['width_px'], color['height_px'],
                               math.radians(color['horizontal_fov_deg']), math.radians(color['vertical_fov_deg']))
            views[name] = {'topic': topic, 'pose_xyz_rpy': pose, 'expected_signal': 'red'}
    else:
        from capture_official_track_views import TRACK_CENTER, VIEWS
        for view in VIEWS:
            x, y, z = view['position']
            dx, dy, dz = [a - b for a, b in zip(TRACK_CENTER, (x, y, z))]
            pose = [x, y, z, 0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)]
            topic = add_camera(world, view['name'], pose, 2300, 1500, 1.2)
            views[view['name']] = {'topic': topic, 'pose_xyz_rpy': pose}
        for marker_id in (20, 30):
            marker = track.find(f"link[@name='aruco_{marker_id}']")
            x, y, z, _, _, yaw = map(float, marker.findtext('pose').split())
            distance, height = 1.2, .65
            pose = [x + distance * math.cos(yaw), y + distance * math.sin(yaw), height,
                    0, math.atan2(height - .30, distance), math.atan2(-math.sin(yaw), -math.cos(yaw))]
            name = f'gantry_{marker_id}_detail'
            topic = add_camera(world, name, pose, 1280, 960, 1.0)
            views[name] = {'topic': topic, 'pose_xyz_rpy': pose}
    assert ET.tostring(track) == original
    added_faces = []
    if args.marker_face == 'sdf_cells':
        from gantry_marker_face import add_sdf_marker_faces
        added_faces = add_sdf_marker_faces(track)
    ET.indent(root)
    ET.ElementTree(root).write(work / 'world.sdf', encoding='utf-8', xml_declaration=True)
    for name in ('aruco', 'meshes'):
        if (source / name).exists():
            shutil.copytree(source / name, work / name)
    os.environ['GZ_PARTITION'] = f'gantry_views_{os.getpid()}'
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 20)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_STATIC_PEERS'] = ''
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = 'true'
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('gantry_views')
    frames, observations = {}, {name: [] for name in views}
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    def receive(message, name):
        if message.encoding != 'rgb8':
            raise ValueError(message.encoding)
        rgb = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        _, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
        frames[name] = rgb
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        red_pixels = int(np.count_nonzero(((hsv[:, :, 0] < 10) | (hsv[:, :, 0] > 170)) &
                                          (hsv[:, :, 1] > 150) & (hsv[:, :, 2] > 150)))
        observations[name].append({'stamp_s': message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                                   'bright_red_pixels': red_pixels,
                                   'ids': [] if ids is None else ids.flatten().tolist(), 'rejected_candidates': len(rejected)})
    subs = [node.create_subscription(Image, view['topic'], lambda message, name=name: receive(message, name),
                                     QoSProfile(depth=2)) for name, view in views.items()]
    commands = {'server': ['gz', 'sim', '-s', '-r', '-v', '4', 'world.sdf'],
                'bridge': ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *[
                    view['topic'] + '@sensor_msgs/msg/Image[gz.msgs.Image' for view in views.values()]]}
    processes, handles = [], []
    report = {'source_world_sha256': sha256(source / 'world.sdf'), 'source_model_unchanged': not added_faces,
              'source_links_unchanged': True, 'marker_face': args.marker_face, 'added_faces': added_faces,
              'diagnostic_world_sha256': sha256(work / 'world.sdf'),
              'source': str(source.relative_to(REPO)), 'mode': args.mode, 'camera': camera,
              'pitch_up_deg': args.pitch_up_deg, 'opencv': cv2.__version__, 'views': views,
              'scope': '차량 카메라 상당 정지 표본. 차체·상대차 가림, 주행, 실물 노출/센서 성능 미검증.',
              'commands': commands, 'requested_frames': 3}
    try:
        for name, cmd in commands.items():
            log = (output / f'{name}.log').open('wb')
            handles.append(log)
            processes.append(subprocess.Popen(cmd, cwd=work, stdout=log, stderr=subprocess.STDOUT,
                                              stdin=subprocess.DEVNULL, start_new_session=True))
        deadline = time.monotonic() + args.timeout
        while min(map(len, observations.values())) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        report['capture_complete'] = min(map(len, observations.values())) >= 3
        for name, rgb in frames.items():
            target = output / f'{name}.png'
            if not cv2.imwrite(str(target), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError(str(target))
            views[name].update(file=target.name, sha256=sha256(target))
        for name in views:
            views[name]['observations'] = observations[name]
            if 'expected_id' in views[name]:
                views[name]['expected_detected_all_frames'] = bool(observations[name]) and all(
                    views[name]['expected_id'] in frame['ids'] for frame in observations[name])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        report['cleanup'] = [stop_test(process) for process in reversed(processes)]
        for handle in handles:
            handle.close()
        report['errors_warnings'] = {}
        for name in commands:
            log = (output / f'{name}.log').read_text(encoding='utf-8', errors='replace')
            clean = re.sub(r'\x1b\[[0-9;]*m', '', log)
            report['errors_warnings'][name] = [line for line in clean.splitlines() if any(t in line for t in ('[Err]', '[Wrn]', 'Exception'))]
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'capture_complete': report.get('capture_complete'), 'cleanup': report['cleanup'],
                      'detections': {name: view.get('expected_detected_all_frames') for name, view in views.items()}}, indent=2))
    return 0 if report.get('capture_complete') and all(p['return_code'] == 0 and not p['remaining_test_pids'] for p in report['cleanup']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
