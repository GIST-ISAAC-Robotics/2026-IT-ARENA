#!/usr/bin/env python3
"""주행 없이 현재 차량 후방 마커를 Gazebo 외부 카메라로 촬영한다.

사진은 센서 원본 PNG이며 합성/후처리를 하지 않는다. 정답 위치는 카메라
배치에만 사용한다. 자율주행·명령 발행·실물 검증은 수행하지 않는다.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time
import zipfile

import cv2
import yaml

from capture_official_track_views import REPO, gazebo_service, image_array, sha256
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch


VIEWS = (
    dict(name='rear_portrait', position=(-.36, 0., .145), target=(-.035, 0., .085), hfov=.90),
    dict(name='rear_three_quarter', position=(-.36, .13, .225), target=(-.015, 0., .080), hfov=.92),
)
WIDTH, HEIGHT = 1600, 2000


def camera_sdf(view, slot, topic):
    cx, cy, cz = view['position']
    tx, ty, tz = view['target']
    yaw = slot['yaw_rad']
    x = slot['x'] + cx * math.cos(yaw) - cy * math.sin(yaw)
    y = slot['y'] + cx * math.sin(yaw) + cy * math.cos(yaw)
    camera_yaw = yaw + math.atan2(ty-cy, tx-cx)
    pitch = math.atan2(cz-tz, math.hypot(tx-cx, ty-cy))
    return f'''<sdf version="1.10"><model name="photo_{view['name']}">
      <static>true</static><pose>{x} {y} {cz} 0 {pitch} {camera_yaw}</pose>
      <link name="camera"><sensor name="rgb" type="camera">
        <always_on>true</always_on><update_rate>2</update_rate><topic>{topic}</topic>
        <camera><horizontal_fov>{view['hfov']}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.01</near><far>40</far></clip>
        </camera>
      </sensor></link></model></sdf>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=150.)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('촬영 자료는 artifacts 아래에 저장합니다.')
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(170+os.getpid()%50),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
                      GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}')
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    world = REPO/'src/arena_gazebo/worlds/it_arena_official'
    slot = next(s for s in json.loads((world/'scene.json').read_text())['starting_grid']['slots'] if s['index'] == 3)
    config = REPO/'src/arena_description/config/vehicle.yaml'
    marker = yaml.safe_load(config.read_text())['vehicle']['identification_marker']
    sources = [Path(__file__), config, world/'world.sdf', world/'scene.json',
               REPO/'src/arena_description/models/arena_car/model.sdf.xacro',
               REPO/'src/arena_description/config/b0183_c1.yaml',
               REPO/'src/arena_bringup/launch/simulation.launch.py',
               REPO/marker['source_image']]
    hashes = {str(p.relative_to(REPO)): sha256(p) for p in sources}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(p, str(p.relative_to(REPO)))
    report = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), passed=False,
                  marker=marker, slot=slot, source_sha256=hashes, views=[],
                  scope='static Gazebo raw camera views; no drive commands or real camera validation')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('rear_marker_photo')
    state = {}
    node.create_subscription(Clock, '/clock', lambda m: state.update(clock=m), 10)
    node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=m), 10)
    log_path = output/'capture.log'
    log = log_path.open('w')
    simulation = None
    bridges = []

    def wait_for(predicate):
        deadline = time.monotonic()+args.timeout
        while not predicate():
            if simulation is not None and simulation.poll() is not None:
                raise RuntimeError('촬영 전 simulation 종료: capture.log 확인')
            if time.monotonic() > deadline:
                raise TimeoutError('촬영 프레임/차량 준비 시간 초과')
            rclpy.spin_once(node, timeout_sec=.05)

    try:
        command = ['ros2', 'launch', '--noninteractive', 'arena_bringup', 'simulation.launch.py',
                   'headless:=true', 'track:=official', 'grid_slot:=3',
                   'autonomy:=false', 'autonomy_mode:=local_pursuit', 'traffic_light:=false',
                   'render_sensors:=true', 'tof_safety:=false', 'depth_camera:=false',
                   'lidar_compensation:=both', 'lidar_acquisition:=sequential', 'sensor_wall_timeout_s:=30',
                   'render_backend:=wsl_nvidia', 'batch_static_visuals:=true', 'speed_bump:=false',
                   f'runtime_artifacts:={output}']
        report['command'] = command
        simulation = subprocess.Popen(command, cwd=REPO, stdin=subprocess.DEVNULL,
                                      stdout=log, stderr=log, start_new_session=True)
        wait_for(lambda: 'odom' in state and 'clock' in state and state['clock'].clock.sec >= 2)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for view in VIEWS:
            topic = '/photo/'+view['name']
            frame = dict(count=0)
            def receive(message):
                frame.update(count=frame['count']+1, message=message)
            subscription = node.create_subscription(Image, topic, receive, qos_profile_sensor_data)
            sdf = camera_sdf(view, slot, topic)
            (output/(view['name']+'.sdf')).write_text(sdf)
            gazebo_service('create', 'gz.msgs.EntityFactory', 'sdf: '+json.dumps(sdf))
            bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                       f'{topic}@sensor_msgs/msg/Image[gz.msgs.Image'],
                                      stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                      start_new_session=True)
            bridges.append(bridge)
            wait_for(lambda: frame['count'] >= 3)
            pixels = image_array(frame['message'])
            filename = view['name']+'.png'
            if pixels.shape[:2] != (HEIGHT, WIDTH) or not cv2.imwrite(str(output/filename), pixels):
                raise RuntimeError('사진 크기/저장 실패')
            gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            found = [] if ids is None else ids.flatten().tolist()
            report['views'].append(dict(**view, file=filename, size_px=[WIDTH, HEIGHT],
                                        sha256=sha256(output/filename), detected_ids=found,
                                        marker_corners_px=[c.tolist() for c in corners]))
            print(json.dumps(dict(file=filename, detected_ids=found)), flush=True)
            gazebo_service('remove', 'gz.msgs.Entity', f'name: "photo_{view["name"]}", type: MODEL')
            node.destroy_subscription(subscription)
            cleanup = stop_launch(bridge)
            if cleanup or bridge.returncode != 0:
                report.setdefault('bridge_cleanup_errors', []).append(dict(cleanup, return_code=bridge.returncode))
        report['capture_passed'] = len(report['views']) == len(VIEWS) and all(marker['id'] in v['detected_ids'] for v in report['views'])
    except Exception as error:
        report['error'] = repr(error)
    finally:
        for bridge in bridges:
            if bridge.poll() is None:
                report.setdefault('bridge_cleanup', []).append(stop_launch(bridge))
        if simulation is not None:
            report['server_stop'] = stop_gazebo_service(simulation, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
            cleanup = stop_launch(simulation)
            report['shutdown'] = audit_shutdown(log_path.read_text(errors='replace'), simulation.returncode, cleanup.get('forced_cleanup'))
            report['remaining_pids'] = list(group_processes(simulation.pid))
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        report['source_unchanged'] = hashes == {str(p.relative_to(REPO)): sha256(p) for p in sources}
        report['passed'] = bool(report.get('capture_passed') and report.get('server_stop', {}).get('verified')
                                and report.get('shutdown', {}).get('clean') and not report.get('remaining_pids')
                                and not report.get('bridge_cleanup_errors') and report['source_unchanged'])
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(dict(passed=report['passed'], error=report.get('error'), output=str(output))), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
