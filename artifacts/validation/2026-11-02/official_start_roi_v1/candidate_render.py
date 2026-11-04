"""가상 상향 시점 후보의 실제 점등 판독과 코스 마커 대표 프레임. 차량 설정·장착·주행과 무관하다.

signal: 여섯 칸 × 시점별로 시험 초록→소등→빨강 1–4→초록을 순서대로 판독한다.
markers: 중심선 위 ID20/30 접근 거리별 정지 프레임의 검출만 본다.
"""
import argparse
import hashlib
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

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE.parent/'official_facility_v1'))
import live_signal_render as base  # noqa: E402
from arena_autonomy.core import image_rgb, marker_ids  # noqa: E402
from audit_grid_start import camera_model  # noqa: E402
from audit_start_signal_geometry import rotation  # noqa: E402
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch  # noqa: E402
from validate_start_signal_render import cleanup_owned_residue, stop_bridge_first  # noqa: E402

PITCHES = (0., 20., 25.)
MARKER_DISTANCES = (3., 2., 1.2, .7)
MOUNT = np.asarray(base.MOUNT['xyz_m'], dtype=float)


def fixture_world():
    root = ET.parse(base.WORLD_DIR/'world.sdf').getroot()
    world = root.find('world')
    for plugin in list(world.findall('plugin')):
        if plugin.get('name') == 'gz::sim::systems::Imu':
            world.remove(plugin)
    for element in world.iter():
        if element.tag in ('albedo_map', 'uri') and element.text and '://' not in element.text:
            element.text = str((base.WORLD_DIR/element.text).resolve())
    return root, world


def add_view(world, views, name, x, y, yaw, pitch):
    xyz = np.array([x, y, base.ROAD_TOP_M])+rotation(0., 0., yaw)@MOUNT
    views[name] = camera_model(world, name, xyz, [0., -math.radians(pitch), yaw], base.CAMERA)


def centerline_pose(s):
    data = np.genfromtxt(base.WORLD_DIR/'centerline.csv', delimiter=',', names=True)
    x, y = data['x_m'], data['y_m']
    distance = np.r_[0., np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    px, py = np.interp([s, s+.02], distance, x), np.interp([s, s+.02], distance, y)
    return float(px[0]), float(py[0]), math.atan2(py[1]-py[0], px[1]-px[0])


def build(mode, target):
    root, world = fixture_world()
    scene = json.loads((base.WORLD_DIR/'scene.json').read_text(encoding='utf-8'))
    views = {}
    if mode == 'signal':
        for slot in scene['starting_grid']['slots']:
            for pitch in PITCHES:
                add_view(world, views, f"slot{slot['index']}_p{pitch:.0f}", slot['x'], slot['y'], slot['yaw_rad'], pitch)
        rate = '10'
    else:
        markers = {m['id']: m for m in scene['aruco_markers']['markers']}
        for marker in (20, 30):
            for distance in MARKER_DISTANCES:
                x, y, yaw = centerline_pose(markers[marker]['s_m']-distance)
                for pitch in PITCHES:
                    add_view(world, views, f'id{marker}_d{int(distance*100)}_p{pitch:.0f}', x, y, yaw, pitch)
        rate = '2'
    for sensor in world.iter('sensor'):
        sensor.find('update_rate').text = rate
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    return views


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('signal', 'markers'))
    parser.add_argument('--timeout', type=float, default=300.)
    args = parser.parse_args()
    output = HERE/f'render_{args.mode}'
    output.mkdir(exist_ok=False)
    fixture = output/'fixture.sdf'
    views = build(args.mode, fixture)
    launch_file = output/'fixture.launch.py'
    launch_file.write_text(base.launch_text(fixture, views), encoding='utf-8')
    paths = [base.WORLD_DIR/'world.sdf', base.CONTROLLER, Path(__file__), ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',
             ROOT/'src/arena_autonomy/arena_autonomy/race_start_local.py']
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    report = dict(passed=False, scope='가상 시점 후보. 차량 기본 수평 설정·공식 모델 불변. 주행·실물 장착 아님.', source_sha256=hashes, views=views)
    os.environ.update(GZ_PARTITION=f'arena_roi_{os.getpid()}', ROS_DOMAIN_ID=str(170+os.getpid() % 20),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    controller = base.load_controller()
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('official_roi_render')
    latest = {name: None for name in views}

    def receive(message, name):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if latest[name] is None or stamp > latest[name]['stamp']:
            latest[name] = dict(stamp=stamp, rgb=image_rgb(message).copy())
    subscriptions = [node.create_subscription(Image, view['topic'], lambda m, n=name: receive(m, n), QoSProfile(depth=2))
                     for name, view in views.items()]
    assert subscriptions
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
        if args.mode == 'signal':
            client = controller.LampClient(str(ROOT/'install/arena_gazebo/lib/arena_gazebo/visual_material_client'),
                                           '/world/it_arena_track/visual_config/blocking')
            report['receipts'] = []
            for index, stage in enumerate(base.STAGES):
                receipts = client.apply(stage) if index else []
                report['receipts'].append(dict(stage=stage, all_applied=all(r['applied'] for r in receipts)))
                floor = max(value['stamp'] for value in latest.values())+.11
                taken = {name: 0 for name in views}
                while min(taken.values()) < base.FRAMES_PER_STAGE:
                    if time.monotonic() > deadline:
                        raise RuntimeError('단계 프레임 시간 초과: '+stage)
                    rclpy.spin_once(node, timeout_sec=.02)
                    for name, value in latest.items():
                        if value['stamp'] >= floor and (not frames[name] or value['stamp'] > frames[name][-1]['stamp']) \
                                and taken[name] < base.FRAMES_PER_STAGE:
                            frames[name].append(dict(stage=f'{index}:{stage}', stamp=value['stamp'], rgb=value['rgb']))
                            taken[name] += 1
            report['views_result'] = {}
            for name, rows in frames.items():
                ids = sorted({i for row in rows for i in marker_ids(row['rgb'], .02)})
                for stage in ('6:red4', '7:green'):
                    row = next(r for r in rows if r['stage'] == stage)
                    cv2.imwrite(str(output/(name+'_'+stage.split(':')[1]+'.png')), cv2.cvtColor(row['rgb'], cv2.COLOR_RGB2BGR))
                detection = base.detect(rows)['LocalOfficialStartSignal']
                report['views_result'][name] = dict(frames=len(rows), started_stage=detection['started_stage'],
                                                    red4=detection['states']['6:red4'][-1], green=detection['states']['7:green'][-1],
                                                    probe=detection['states']['1:green_probe'][-1], markers=ids)
        else:
            floor = max(value['stamp'] for value in latest.values())+.6
            while min(value['stamp'] for value in latest.values()) < floor:
                if time.monotonic() > deadline:
                    raise RuntimeError('마커 프레임 시간 초과')
                rclpy.spin_once(node, timeout_sec=.02)
            report['views_result'] = {}
            for name, value in latest.items():
                cv2.imwrite(str(output/(name+'.png')), cv2.cvtColor(value['rgb'], cv2.COLOR_RGB2BGR))
                report['views_result'][name] = dict(markers=sorted(set(marker_ids(value['rgb'], .02))))
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
            report['bridge_shutdown'] = stop_bridge_first(process, log_path.read_text(errors='replace'))
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
