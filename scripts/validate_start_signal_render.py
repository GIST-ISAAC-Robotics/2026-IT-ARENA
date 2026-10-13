#!/usr/bin/env python3
"""B3 광원 RGB 재질 시험. 독립 정적 장면의 영상을 재생하며 실제 신호 전환 시험은 아니다."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import zipfile

import cv2
import numpy as np
import yaml

from audit_start_signal_geometry import lamps_from_sdf, diagnostic_transform, rotation
from arena_autonomy.core import image_rgb
from arena_autonomy.race_start import OfficialStartSignal
from lidar_shutdown import stop_gazebo_service, stop_launch, audit_shutdown, group_processes

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'artifacts/validation/2026-10-13/race_start_geometry_v1/result'
STAGES = ('off', 'red4', 'green')


def put(parent, tag, text, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    child.text = str(text)
    return child


def build_fixture(scene, lamps, camera):
    root = ET.Element('sdf', version='1.8')
    world = ET.SubElement(root, 'world', name='b3_optical_fixture')
    physics = ET.SubElement(world, 'physics', name='physics', type='ode')
    put(physics, 'max_step_size', .001)
    put(physics, 'real_time_factor', 1.)
    for plugin in ('physics', 'user-commands', 'scene-broadcaster', 'sensors'):
        names = {'physics': 'Physics', 'user-commands': 'UserCommands',
                 'scene-broadcaster': 'SceneBroadcaster', 'sensors': 'Sensors'}
        item = ET.SubElement(world, 'plugin', filename=f'gz-sim-{plugin}-system', name='gz::sim::systems::'+names[plugin])
        if plugin == 'sensors':
            put(item, 'render_engine', 'ogre2')
    lighting = ET.SubElement(world, 'scene')
    put(lighting, 'ambient', '0.4 0.4 0.4 1')
    put(lighting, 'background', '0.2 0.2 0.2 1')
    lamps = diagnostic_transform(lamps, yaw_delta=-math.pi/2, z=.30)
    views = {}
    # 가장 가까운/먼 그리드 2개만. 각 정적 장면을 100 m 격리하며 far clip은 10 m.
    for index in (2, 3):
        slot = next(s for s in scene['starting_grid']['slots'] if s['index'] == index)
        for stage in STAGES:
            name = f'grid{index}_{stage}'
            offset = np.array([100.*len(views), 0., 0.])
            model = ET.SubElement(world, 'model', name='lamps_'+name)
            put(model, 'static', 'true')
            for lamp in lamps:
                link = ET.SubElement(model, 'link', name=lamp['name'])
                put(link, 'pose', ' '.join(map(str, [*(np.asarray(lamp['xyz'])+offset), 0, 0, 0])))
                visual = ET.SubElement(link, 'visual', name='visual')
                sphere = ET.SubElement(ET.SubElement(visual, 'geometry'), 'sphere')
                put(sphere, 'radius', lamp['radius'])
                is_green = lamp['name'] == 'lamp_green'
                on = stage == ('green' if is_green else 'red4')
                color = ('0 1 0 1' if is_green else '1 0 0 1') if on else '.004 .004 .004 1'
                material = ET.SubElement(visual, 'material')
                for field in ('ambient', 'diffuse'):
                    put(material, field, color)
                put(material, 'emissive', color if on else '0 0 0 1')
            camera_model = ET.SubElement(world, 'model', name='camera_'+name)
            put(camera_model, 'static', 'true')
            xyz = np.array([slot['x'], slot['y'], 0.])+rotation(0, 0, slot['yaw_rad'])@camera['xyz_m']+offset
            pose = [*xyz, *camera['rpy_rad'][:2], slot['yaw_rad']+camera['rpy_rad'][2]]
            put(camera_model, 'pose', ' '.join(map(str, pose)))
            sensor = ET.SubElement(ET.SubElement(camera_model, 'link', name='link'), 'sensor', name='rgb', type='camera')
            put(sensor, 'always_on', 'true')
            put(sensor, 'update_rate', 10)
            topic = '/b3_optical/'+name
            put(sensor, 'topic', topic)
            cam = ET.SubElement(sensor, 'camera')
            put(cam, 'horizontal_fov', math.radians(camera['horizontal_fov_deg']))
            image = ET.SubElement(cam, 'image')
            for key, value in (('width', camera['width_px']), ('height', camera['height_px']), ('format', 'R8G8B8')):
                put(image, key, value)
            clip = ET.SubElement(cam, 'clip')
            put(clip, 'near', .02)
            put(clip, 'far', 10)
            views[name] = dict(topic=topic, grid_index=index, stage=stage, camera_pose=list(map(float, pose)))
    return root, views


def replay_pixels(frames):
    """정적 촬영 3단계를 0.1 s 간격으로 연결한다. 원래 촬영 시각/광원 전환을 재현하지 않는다."""
    output = {}
    for index in (2, 3):
        detector = OfficialStartSignal()
        t, rows = 0., []
        for stage in ('green', 'off', 'red4', 'green', 'off'):
            for frame in frames[f'grid{index}_{stage}']:
                before = detector.started
                detector.update(frame['rgb'], round(t, 6))
                rows.append(dict(stage=stage, assigned_replay_stamp=t, captured_stamp=frame['stamp'],
                                 started=detector.started, observed=detector.observed,
                                 new_start=bool(detector.started and not before)))
                t += .1
        output[str(index)] = dict(rows=rows, early_start=any(r['started'] for r in rows[:9]),
            green_detected=any(r['started'] for r in rows[9:12]), off_keeps_start=all(r['started'] for r in rows[12:]))
    return output


def owned_bridge(process_id, log, rows):
    matches = re.findall(r'\[parameter_bridge-\d+\]: process started with pid \[(\d+)\]', log)
    if len(matches) != 1:
        raise ValueError('native bridge PID not unique')
    pid = int(matches[0])
    identity = rows.get(pid)
    if identity is None or identity['ppid'] != process_id:
        raise ValueError('bridge not a current owned child')
    return pid, identity


def stop_bridge_first(process, log):
    result = dict(verified=False)
    if process is None or process.poll() is not None:
        return dict(result, error='launch_not_running')
    try:
        pid, identity = owned_bridge(process.pid, log, group_processes(process.pid))
        result.update(pid=pid, identity=identity)
        if group_processes(process.pid).get(pid) != identity:
            raise ValueError('bridge identity changed before shutdown')
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic()+8.
        while group_processes(process.pid).get(pid) == identity and time.monotonic() < deadline:
            time.sleep(.05)
        result['verified'] = group_processes(process.pid).get(pid) != identity
    except Exception as error:
        result['error'] = repr(error)
    return result


def cleanup_owned_residue(process_id, before):
    """정상 종료 실패 뒤에도 미리 기록한 동일 PID/시작 시각만 정리한다."""
    actions = []
    for sig in (signal.SIGTERM, signal.SIGKILL):
        rows = group_processes(process_id)
        owned = {pid: row for pid, row in rows.items() if pid in before and row['start'] == before[pid]['start']}
        if not owned:
            break
        for pid, row in owned.items():
            current = group_processes(process_id).get(pid)
            if current is not None and current['start'] == row['start']:
                try:
                    os.kill(pid, sig)
                    actions.append(dict(pid=pid, start=row['start'], signal=sig.name))
                except ProcessLookupError:
                    pass
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            rows = group_processes(process_id)
            if not any(pid in rows and rows[pid]['start'] == row['start'] for pid,row in owned.items()):
                break
            time.sleep(.05)
    return dict(actions=actions, remaining=group_processes(process_id))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts' or not 30 <= args.timeout <= 300:
        parser.error('fresh artifact path and 30..300 s timeout required')
    output.mkdir(parents=True, exist_ok=False)
    paths = ['scripts/validate_start_signal_render.py', 'scripts/audit_start_signal_geometry.py',
             'scripts/lidar_shutdown.py', 'src/arena_autonomy/arena_autonomy/race_start.py',
             'src/arena_autonomy/arena_autonomy/core.py', 'src/arena_description/config/b0183_c1.yaml']
    report = dict(passed=False, scope='isolated_static_lamp_pixels_reordered_not_live_signal_or_grid_occlusion',
                  source_hashes={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
                  official_source_hashes={p: hashlib.sha256((SOURCE/p).read_bytes()).hexdigest() for p in ('scene.json', 'world.sdf')})
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            bundle.write(ROOT/path, path)
    scene = json.loads((SOURCE/'scene.json').read_text())
    camera = yaml.safe_load((ROOT/'src/arena_description/config/b0183_c1.yaml').read_text())['camera']
    root, views = build_fixture(scene, lamps_from_sdf((SOURCE/'world.sdf').read_bytes()), camera)
    ET.indent(root)
    ET.ElementTree(root).write(output/'fixture.sdf', encoding='utf-8', xml_declaration=True)
    report['views'] = views
    os.environ.update(GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}', ROS_DOMAIN_ID=str(170+os.getpid()%20),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='', LIBGL_ALWAYS_SOFTWARE='true',
                      GALLIUM_DRIVER='llvmpipe')
    bridge_arguments = [v['topic']+'@sensor_msgs/msg/Image[gz.msgs.Image' for v in views.values()]
    server = ['gz', 'sim', '-s', '-r', '-v', '4', str(output/'fixture.sdf')]
    # launch 로그로 시작한 자식의 정상 종료를 별도 감사한다.
    launch = ('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
              'from launch_ros.actions import Node\n'
              'def generate_launch_description():\n    return LaunchDescription(['+
              f'ExecuteProcess(cmd={server!r}, output="screen"), '+
              f'Node(package="ros_gz_bridge", executable="parameter_bridge", arguments={bridge_arguments!r}, output="screen")])\n')
    (output/'fixture.launch.py').write_text(launch, encoding='utf-8')
    import rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import QoSProfile
    rclpy.init()
    node = rclpy.create_node('b3_optical_capture')
    frames = {name: [] for name in views}
    def receive(message, name):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if len(frames[name]) < 3 and (not frames[name] or stamp > frames[name][-1]['stamp']):
            frames[name].append(dict(stamp=stamp, rgb=image_rgb(message).copy()))
    subscriptions = [node.create_subscription(Image, view['topic'],
                     lambda msg, name=name: receive(msg, name), QoSProfile(depth=2)) for name, view in views.items()]
    process = None
    log_path = output/'launch.log'
    log = log_path.open('wb')
    try:
        process = subprocess.Popen(['ros2', 'launch', str(output/'fixture.launch.py')], cwd=output,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+args.timeout
        while min(map(len, frames.values())) < 3 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('fixture launch exited before capture')
            rclpy.spin_once(node, timeout_sec=.1)
        report['captured_counts'] = {name: len(value) for name, value in frames.items()}
        report['capture_complete'] = min(map(len, frames.values())) == 3
        for name, values in frames.items():
            if values:
                if not cv2.imwrite(str(output/(name+'.png')), cv2.cvtColor(values[-1]['rgb'], cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('image save failed')
        if not report['capture_complete']:
            raise RuntimeError('capture deadline or incomplete distinct frames')
        report['pixel_replay'] = replay_pixels(frames)
        report['detection_passed'] = all(not r['early_start'] and r['green_detected'] and r['off_keeps_start']
                                         for r in report['pixel_replay'].values())
    except Exception as error:
        report['error'] = repr(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        before = group_processes(process.pid) if process else {}
        report['owned_before_shutdown'] = before
        report['bridge_shutdown'] = stop_bridge_first(process, log_path.read_text(errors='replace'))
        report['gazebo_stop_service'] = stop_gazebo_service(process, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
        cleanup = stop_launch(process)
        log.close()
        report['shutdown_audit'] = audit_shutdown(log_path.read_text(errors='replace'),
                                                 process.poll() if process else None, cleanup or None)
        report['residue_cleanup'] = cleanup_owned_residue(process.pid, before) if process else dict(actions=[], remaining={})
        report['remaining_test_pids'] = list(report['residue_cleanup']['remaining'])
        report['source_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p,h in report['source_hashes'].items())
        report['passed'] = bool(report.get('capture_complete') and report.get('detection_passed') and
                                 report['bridge_shutdown']['verified'] and report['gazebo_stop_service']['verified'] and
                                 report['shutdown_audit']['clean'] and not report['residue_cleanup']['actions'] and
                                 not report['remaining_test_pids'] and report['source_unchanged'] and 'error' not in report)
        (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: report.get(k) for k in ('passed', 'capture_complete', 'detection_passed', 'error', 'remaining_test_pids')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
