#!/usr/bin/env python3
"""M5 사전 생성 정지 장면 촬영. 제어기·MCU·명령 토픽을 실행하지 않는다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2

from arena_autonomy.core import image_rgb
from audit_grid_start import ROOT
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch
from validate_start_signal_render import stop_bridge_first, cleanup_owned_residue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('preflight', type=Path)
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    source = args.preflight.resolve()
    if not source.is_relative_to(ROOT/'artifacts') or not 30 <= args.timeout <= 300:
        parser.error('artifacts preflight and bounded timeout required')
    spec = json.loads((source/'report.json').read_text(encoding='utf-8'))
    fixture = source/'fixture.sdf'
    world = ET.parse(fixture).find('world')
    if (world.get('name') != 'm5_grid_fixture' or len(world.findall('model')) != 9
            or any(m.findtext('static') != 'true' for m in world.findall('model'))
            or len(list(world.iter('plugin'))) != 4):
        raise ValueError('only preregistered static M5 fixture is allowed')
    output = source/'capture'
    output.mkdir(exist_ok=False)
    paths = [fixture, source/'report.json', Path(__file__), ROOT/'scripts/lidar_shutdown.py',
             ROOT/'scripts/validate_start_signal_render.py', ROOT/'scripts/audit_grid_start.py',
             ROOT/'src/arena_autonomy/arena_autonomy/core.py']
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    views = spec['render_views']
    report = dict(passed=False, scope='static five nominal box cars, official lamps unchanged; no driving validation',
                  source_sha256=hashes, views=views, control_processes_launched=False)
    os.environ.update(GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}', ROS_DOMAIN_ID=str(170+os.getpid()%20),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
                      LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe')
    server = ['gz', 'sim', '-s', '-r', '-v', '3', str(fixture)]
    arguments = [v['topic']+'@sensor_msgs/msg/Image[gz.msgs.Image' for v in views.values()]
    launch = ('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
              'from launch_ros.actions import Node\n'
              'def generate_launch_description():\n    return LaunchDescription(['+
              f'ExecuteProcess(cmd={server!r}, output="screen"), '+
              f'Node(package="ros_gz_bridge", executable="parameter_bridge", arguments={arguments!r}, output="screen")])\n')
    (output/'fixture.launch.py').write_text(launch, encoding='utf-8')
    import rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import QoSProfile
    rclpy.init()
    node = rclpy.create_node('m5_grid_capture')
    frames = {name: [] for name in views}
    def receive(message, name):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if len(frames[name]) < 3 and (not frames[name] or stamp > frames[name][-1]['stamp']):
            frames[name].append(dict(stamp=stamp, rgb=image_rgb(message).copy()))
    subscriptions = [node.create_subscription(Image, v['topic'], lambda msg, n=n: receive(msg, n), QoSProfile(depth=2))
                     for n, v in views.items()]
    process = None
    log_path = output/'launch.log'
    log = log_path.open('wb')
    try:
        process = subprocess.Popen(['ros2', 'launch', str(output/'fixture.launch.py')], cwd=output,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+args.timeout
        while min(map(len, frames.values())) < 3 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('fixture exited before capture')
            rclpy.spin_once(node, timeout_sec=.1)
        report['frame_counts'] = {n: len(v) for n, v in frames.items()}
        report['capture_complete'] = all(len(v) == 3 for v in frames.values())
        report['images'] = []
        for name, values in frames.items():
            if values:
                filename = output/(name+'.png')
                rgb = values[-1]['rgb']
                expected = views[name]['camera']
                if rgb.shape != (expected['height_px'], expected['width_px'], 3):
                    raise ValueError('unexpected camera dimensions')
                if not cv2.imwrite(str(filename), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('image save failed')
                report['images'].append(dict(file=filename.name, sha256=hashlib.sha256(filename.read_bytes()).hexdigest(),
                                             stamps=[v['stamp'] for v in values], shape=list(rgb.shape)))
        if not report['capture_complete']:
            raise RuntimeError('incomplete capture')
    except Exception as error:
        report['error'] = repr(error)
    finally:
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
        report['source_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p,h in hashes.items())
        report['passed'] = bool(report.get('capture_complete') and report.get('bridge_shutdown', {}).get('verified')
            and report.get('server_stop', {}).get('verified') and report.get('shutdown', {}).get('clean')
            and not report.get('residue_cleanup', {}).get('actions')
            and not report.get('residue_cleanup', {}).get('remaining') and report['source_unchanged'] and 'error' not in report)
        (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: report.get(k) for k in ('passed', 'capture_complete', 'error', 'source_unchanged')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
