"""새 실행 월드를 실제 Gazebo로 띄워 공식 2×5 광원을 전환하고 차량 카메라 위치의 프레임을 검출기에 넣는다.

주행·MCU·제어기는 실행하지 않는다. slot*는 현 임시 장착(수평), *_up15는 진단용 가상 피치이며 차량 변경이 아니다.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy')]
from arena_autonomy.core import image_rgb, marker_ids  # noqa: E402
from arena_autonomy.race_start import OfficialStartSignal  # noqa: E402
from arena_autonomy.race_start_local import LocalOfficialStartSignal  # noqa: E402
from audit_grid_start import camera_model  # noqa: E402
from audit_start_signal_geometry import rotation  # noqa: E402
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch  # noqa: E402
from validate_start_signal_render import cleanup_owned_residue, stop_bridge_first  # noqa: E402

WORLD_DIR = ROOT/'src/arena_gazebo/worlds/it_arena_official'
CONTROLLER = ROOT/'src/arena_gazebo/scripts/traffic_light_controller.py'
MOUNT = yaml.safe_load((ROOT/'src/arena_description/config/b0183_c1.yaml').read_text(encoding='utf-8'))['camera']
CAMERA = {k: MOUNT[k] for k in ('width_px', 'height_px', 'horizontal_fov_deg')}
ROAD_TOP_M = .003
STAGES = ('off', 'green_probe', 'off', 'red1', 'red2', 'red3', 'red4', 'green')
FRAMES_PER_STAGE = 6
DIAGNOSTIC_PITCH = {'slot2_up15': (2, 15.), 'slot5_up15': (5, 15.), 'slot3_up15': (3, 15.)}


def load_controller():
    spec = importlib.util.spec_from_file_location('traffic_light_controller_for_render', CONTROLLER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_fixture(target):
    root = ET.parse(WORLD_DIR/'world.sdf').getroot()
    world = root.find('world')
    scene = json.loads((WORLD_DIR/'scene.json').read_text(encoding='utf-8'))
    for plugin in list(world.findall('plugin')):
        if plugin.get('name') == 'gz::sim::systems::Imu':
            world.remove(plugin)
    for element in world.iter():
        if element.tag in ('albedo_map', 'uri') and element.text and '://' not in element.text:
            asset = (WORLD_DIR/element.text).resolve()
            assert asset.is_file(), asset
            element.text = str(asset)
    views = {}
    slots = {slot['index']: slot for slot in scene['starting_grid']['slots']}
    mount = np.asarray(MOUNT['xyz_m'], dtype=float)
    def add(name, slot, pitch_deg):
        xyz = np.array([slot['x'], slot['y'], ROAD_TOP_M])+rotation(0., 0., slot['yaw_rad'])@mount
        views[name] = camera_model(world, name, xyz, [0., -math.radians(pitch_deg), slot['yaw_rad']], CAMERA)
    for index, slot in slots.items():
        add(f'slot{index}', slot, 0.)
    for name, (index, pitch) in DIAGNOSTIC_PITCH.items():
        add(name, slots[index], pitch)
    xyz = np.array([9.2, 5.0, .9])
    delta = np.array([9.9636, 6.1048, .419])-xyz
    views['overview'] = camera_model(world, 'overview', xyz,
        [0., math.atan2(-delta[2], math.hypot(*delta[:2])), math.atan2(delta[1], delta[0])], CAMERA)
    for model in world.findall('model'):
        for sensor in model.iter('sensor'):
            sensor.find('update_rate').text = '10'
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    return views


def launch_text(fixture, views):
    library = str(ROOT/'install/arena_gazebo/lib')
    env = {'GALLIUM_DRIVER': 'd3d12', 'LIBGL_ALWAYS_SOFTWARE': 'false', 'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA',
           'LD_LIBRARY_PATH': library+':'+os.environ.get('LD_LIBRARY_PATH', ''), 'LD_PRELOAD': 'libarena-wsl-d3d12-lifetime.so'}
    server = ['gz', 'sim', '-s', '-r', '-v', '3', '--headless-rendering', str(fixture)]
    bridges = [view['topic']+'@sensor_msgs/msg/Image[gz.msgs.Image' for view in views.values()]
    return ('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
            'from launch_ros.actions import Node\n'
            'def generate_launch_description():\n    return LaunchDescription(['
            f'ExecuteProcess(cmd={server!r}, additional_env={env!r}, output="screen"), '
            f'Node(package="ros_gz_bridge", executable="parameter_bridge", arguments={bridges!r}, output="screen")])\n')


def detect(rows):
    """프레임 순서 그대로 검출기를 돌린다. 초록 시험점등 뒤 네 빨강을 거쳐야 출발 인정."""
    result = {}
    for detector in (OfficialStartSignal, LocalOfficialStartSignal):
        signal = detector()
        states, started_stage = {}, None
        for row in rows:
            signal.update(row['rgb'], row['stamp'])
            states.setdefault(row['stage'], []).append(signal.observed)
            if signal.started and started_stage is None:
                started_stage = row['stage']
        result[detector.__name__] = dict(started_stage=started_stage, states=states)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'render_v1')
    parser.add_argument('--timeout', type=float, default=300.)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(exist_ok=False)
    fixture = output/'fixture.sdf'
    views = build_fixture(fixture)
    launch_file = output/'fixture.launch.py'
    launch_file.write_text(launch_text(fixture, views), encoding='utf-8')
    paths = [WORLD_DIR/'world.sdf', WORLD_DIR/'scene.json', CONTROLLER, Path(__file__),
             ROOT/'src/arena_autonomy/arena_autonomy/race_start.py', ROOT/'src/arena_autonomy/arena_autonomy/race_start_local.py']
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    report = dict(passed=False, scope='static vehicle-mount viewpoints on the regenerated world with live material switching; '
                  'no vehicle/controller/driving; *_up15 is a diagnostic pitch, not a vehicle change',
                  stages=STAGES, source_sha256=hashes, views=views)
    os.environ.update(GZ_PARTITION=f'arena_facility_{os.getpid()}', ROS_DOMAIN_ID=str(170+os.getpid() % 20),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    controller = load_controller()
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('official_facility_render')
    latest = {name: None for name in views}
    def receive(message, name):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if latest[name] is None or stamp > latest[name]['stamp']:
            latest[name] = dict(stamp=stamp, rgb=image_rgb(message).copy())
    subscriptions = [node.create_subscription(Image, view['topic'], lambda m, n=name: receive(m, n), QoSProfile(depth=2))
                     for name, view in views.items()]
    frames = {name: [] for name in views}
    log_path = output/'launch.log'
    log = log_path.open('wb')
    process = client = None
    try:
        process = subprocess.Popen(['ros2', 'launch', str(launch_file)], cwd=output, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+args.timeout
        while any(value is None for value in latest.values()):
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('첫 카메라 프레임 전 종료/시간 초과')
            rclpy.spin_once(node, timeout_sec=.05)
        client = controller.LampClient(str(ROOT/'install/arena_gazebo/lib/arena_gazebo/visual_material_client'),
                                       '/world/it_arena_track/visual_config/blocking')
        report['receipts'] = []
        for index, stage in enumerate(STAGES):
            receipts = client.apply(stage) if index else []
            report['receipts'].append(dict(stage=stage, lamps=len(receipts),
                                           lit=sum(r['lit'] for r in receipts), all_applied=all(r['applied'] for r in receipts)))
            # 적용 시각 뒤 렌더된 프레임만 사용한다.
            floor = max(value['stamp'] for value in latest.values())+.11
            taken = {name: 0 for name in views}
            while min(taken.values()) < FRAMES_PER_STAGE:
                if time.monotonic() > deadline:
                    raise RuntimeError('단계 프레임 시간 초과: '+stage)
                rclpy.spin_once(node, timeout_sec=.02)
                for name, value in latest.items():
                    if value['stamp'] >= floor and (not frames[name] or value['stamp'] > frames[name][-1]['stamp']) \
                            and taken[name] < FRAMES_PER_STAGE:
                        frames[name].append(dict(stage=f'{index}:{stage}', stamp=value['stamp'], rgb=value['rgb']))
                        taken[name] += 1
        report['views_result'] = {}
        for name, rows in frames.items():
            markers = {}
            for row in rows:
                markers.setdefault(row['stage'], set()).update(marker_ids(row['rgb'], .02))
            for stage in ('6:red4', '7:green', '1:green_probe', '2:off'):
                row = next(r for r in rows if r['stage'] == stage)
                file = output/(name+'_'+stage.split(':')[1]+'.png')
                cv2.imwrite(str(file), cv2.cvtColor(row['rgb'], cv2.COLOR_RGB2BGR))
            report['views_result'][name] = dict(frames=len(rows), detection=detect(rows),
                                                markers={k: sorted(v) for k, v in markers.items()})
        report['capture_complete'] = True
    except Exception as error:
        report['error'] = repr(error)
    finally:
        if client is not None:
            client.close()
        node.destroy_node()
        rclpy.shutdown()
        before = group_processes(process.pid) if process else {}
        if process:
            text = log_path.read_text(errors='replace')
            report['bridge_shutdown'] = stop_bridge_first(process, text)
            report['server_stop'] = stop_gazebo_service(process, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
            cleanup = stop_launch(process)
            log.close()
            report['shutdown'] = audit_shutdown(log_path.read_text(errors='replace'), process.poll(), cleanup or None)
            report['residue_cleanup'] = cleanup_owned_residue(process.pid, before)
        else:
            log.close()
        report['source_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
        report['passed'] = bool(report.get('capture_complete') and report['source_unchanged']
                                and report.get('shutdown', {}).get('clean') and 'error' not in report)
        (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str), encoding='utf-8')
    print(json.dumps({k: report.get(k) for k in ('passed', 'capture_complete', 'error', 'source_unchanged')}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
