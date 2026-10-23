"""정지 벽의 명목 광선/최근접 cubemap 광선을 구별하는 1회 진단. 운영 미연결."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time
import xml.etree.ElementTree as ET
import zipfile

import yaml
from validate_bump_lidar_pose import put
from lidar_shutdown import stop_gazebo_service, stop_launch, audit_shutdown, group_processes
from validate_start_signal_render import stop_bridge_first, cleanup_owned_residue

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
RUN = OUT/'result'


def vector(angle, raster=False):
    if abs(angle) >= math.pi/4:
        raise ValueError('front cubemap face only')
    if not raster:
        return math.cos(angle), math.sin(angle), 0.
    # 원문 SampleCubemap의 전방 +Z 면은 u=(1-tan(h))/2다.
    # UV texture는 float32. 정면 경계의 반올림도 보존한다.
    u = struct.unpack('f', struct.pack('f', (1-math.tan(angle))/2))[0]
    q = 1-2*(math.floor(u*128)+.5)/128
    norm = math.sqrt(1+q*q+1/128**2)
    return 1/norm, q/norm, -1/(128*norm)


def expected(case, angle, raster=False):
    x, y, z = vector(angle, raster)
    candidates = []
    if case in ('straight_corridor', 'end_wall'):
        if y != 0:
            candidates.append((.225/abs(y), 'side'))
        if case == 'end_wall':
            candidates.append((3./x, 'front'))
    elif case == 'angled_wall':
        dot = math.cos(math.pi/6)*x+math.sin(math.pi/6)*y
        if dot > 0:
            candidates.append((2./dot, 'angled'))
    else:
        raise ValueError('unknown scene')
    if z < 0:
        candidates.append((.095/-z, 'floor'))
    distance, surface = min(candidates, default=(math.inf, 'none'))
    # 이 전방 부채꼴에서 모든 벽 교점은 30 m 폭의 면 내부다.
    if not .05 <= distance < 12.:
        return None, 'none'
    return distance, surface


def box(world, name, xyz, size, yaw=0.):
    model = ET.SubElement(world, 'model', name=name)
    put(model, 'static', 'true')
    put(model, 'pose', ' '.join(str(v) for v in (*xyz, 0, 0, yaw)))
    link = ET.SubElement(model, 'link', name='link')
    for kind in ('visual', 'collision'):
        item = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
        put(geometry, 'size', ' '.join(map(str, size)))


def fixture(sensor):
    root = ET.Element('sdf', version='1.10')
    world = ET.SubElement(root, 'world', name='wall_raster')
    physics = ET.SubElement(world, 'physics', name='physics', type='ode')
    put(physics, 'max_step_size', .001)
    put(physics, 'real_time_factor', 1.)
    for suffix, name in [('physics', 'Physics'), ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        p = ET.SubElement(world, 'plugin', filename=f'gz-sim-{suffix}-system', name=f'gz::sim::systems::{name}')
        if suffix == 'sensors':
            put(p, 'render_engine', 'ogre2')
    floor = ET.SubElement(world, 'model', name='floor')
    put(floor, 'static', 'true')
    link = ET.SubElement(floor, 'link', name='link')
    for kind in ('visual', 'collision'):
        plane = ET.SubElement(ET.SubElement(ET.SubElement(link, kind, name=kind), 'geometry'), 'plane')
        put(plane, 'normal', '0 0 1')
        put(plane, 'size', '200 200')
    views = {}
    for index, case in enumerate(('straight_corridor', 'end_wall', 'angled_wall')):
        offset = (index-1)*30.
        model = ET.SubElement(world, 'model', name=case)
        put(model, 'static', 'true')
        put(model, 'pose', f'0 {offset} .095 0 0 0')
        holder = ET.SubElement(model, 'link', name='sensor_link')
        clone = copy.deepcopy(sensor)
        clone.find('topic').text = '/audit/wall_raster/'+case
        holder.append(clone)
        views[case] = dict(topic=clone.findtext('topic'), y_offset_m=offset)
        if case != 'angled_wall':
            for side in (-1, 1):
                box(world, f'{case}_side_{side}', (0, offset+side*.25, .2), (30, .05, .4))
            if case == 'end_wall':
                box(world, 'front', (3.025, offset, .2), (.05, .45, .4))
        else:
            phi = math.pi/6
            box(world, 'angled', (2.025*math.cos(phi), offset+2.025*math.sin(phi), .2), (.05, 30, .4), phi)
    return root, views


def compare(frames):
    rows = []
    for case, scans in frames.items():
        for frame, scan in enumerate(scans):
            assert len(scan['ranges']) == 500
            assert abs(scan['angle_min']+math.pi) < 1e-5
            assert abs(scan['angle_increment']-2*math.pi/500) < 1e-6
            assert abs(scan['range_min']-.05) < 1e-6 and scan['range_max'] == 12.
            for index, value in enumerate(scan['ranges']):
                angle = scan['angle_min']+index*scan['angle_increment']
                if abs(angle) > math.radians(44):
                    continue
                nominal, surface = expected(case, angle)
                raster, raster_surface = expected(case, angle, True)
                measured = value if value is not None and value < 12. else None
                range_error = abs(measured-nominal) if measured is not None and nominal is not None else None
                raster_error = abs(measured-raster) if measured is not None and raster is not None else None
                # 소비자가 명목 각도로 재투영했을 때 벽의 법선 방향 잔차.
                normal_error = None
                if measured is not None:
                    if raster_surface == 'side':
                        normal_error = abs(abs(measured*math.sin(angle))-.225)
                    elif raster_surface == 'front':
                        normal_error = abs(measured*math.cos(angle)-3.)
                    elif raster_surface == 'angled':
                        normal_error = abs(measured*math.cos(angle-math.pi/6)-2.)
                rows.append(dict(scene=case, frame=frame, index=index, angle_rad=angle,
                    measured_m=measured, nominal_m=nominal, raster_m=raster,
                    nominal_surface=surface, raster_surface=raster_surface,
                    nominal_hit_agrees=(measured is None) == (nominal is None),
                    raster_hit_agrees=(measured is None) == (raster is None),
                    nominal_range_error_m=range_error, raster_residual_m=raster_error,
                    nominal_angle_wall_normal_residual_m=normal_error))
    summaries = {}
    for case in frames:
        selected = [r for r in rows if r['scene'] == case]
        maxima = lambda key: max((r[key] for r in selected if r[key] is not None), default=None)
        summaries[case] = dict(compared=len(selected),
            nominal_hit_mismatches=sum(not r['nominal_hit_agrees'] for r in selected),
            raster_hit_mismatches=sum(not r['raster_hit_agrees'] for r in selected),
            maximum_nominal_range_error_m=maxima('nominal_range_error_m'),
            maximum_raster_residual_m=maxima('raster_residual_m'),
            maximum_wall_normal_residual_m=maxima('nominal_angle_wall_normal_residual_m'),
            nominal_range_errors_above_30mm=sum(r['nominal_range_error_m'] is not None and r['nominal_range_error_m'] > .03 for r in selected))
    return dict(rows=rows, scenes=summaries, raster_explanation_matches=bool(rows) and all(
        r['raster_hit_agrees'] and (r['raster_residual_m'] is None or r['raster_residual_m'] < .001) for r in rows),
        comparison_tolerance_m=.001, tolerance_is_production_threshold=False)


def main():
    # 독립적인 간단한 기하 확인. GPU/실물의 합격 조건이 아니다.
    assert expected('end_wall', 0.) == (3., 'front')
    assert expected('straight_corridor', 0.) == (None, 'none')
    assert math.isclose(expected('angled_wall', math.pi/6)[0], 2., abs_tol=1e-12)
    assert expected('straight_corridor', .025)[0] < 10.
    RUN.mkdir(exist_ok=False)
    (RUN/'.gitignore').write_text('scan_frames.json\n', encoding='ascii')
    paths = [Path(__file__), ROOT/'scripts/validate_bump_lidar_pose.py', ROOT/'scripts/lidar_shutdown.py',
             ROOT/'scripts/validate_start_signal_render.py', ROOT/'src/arena_description/config/vehicle.yaml',
             ROOT/'src/arena_description/models/arena_car/model.sdf.xacro']
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with zipfile.ZipFile(RUN/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for p in paths:
            bundle.write(p, p.relative_to(ROOT))
    import xacro
    lidar = yaml.safe_load(paths[-2].read_text())['vehicle']['sensors']['lidar_2d']
    mapping = dict(lidar_samples=str(lidar['samples_per_scan']), lidar_min=str(lidar['range_min_m']),
        lidar_max=str(lidar['range_max_m']), lidar_resolution=str(lidar['nominal_range_resolution_m']),
        lidar_noise=str(lidar['simulated_noise_stddev_m']), lidar_rate='10.0')
    generated = ET.fromstring(xacro.process_file(str(paths[-1]), mappings=mapping).toxml())
    sensor = generated.find(".//sensor[@name='rplidar_c1']")
    assert sensor is not None
    root, views = fixture(sensor)
    ET.indent(root)
    ET.ElementTree(root).write(RUN/'fixture.sdf', encoding='utf-8', xml_declaration=True)
    report = dict(scope='static_front_face_wall_raster_diagnostic_not_driving_validation',
        real_sensor_verified=False, controller_present=False, source_hashes=hashes,
        source_sensor_mapping=mapping, views=views,
        raster_reference='https://github.com/gazebosim/gz-rendering/blob/gz-rendering8_8.2.3/ogre2/src/Ogre2GpuRays.cc',
        raster_texture_size=128, raster_uv_precision='float32',
        limits=['front cubemap face +/-44 degrees only', 'three isolated static scenes, no body or motion',
                'no real C1 accuracy claim; no correction applied',
                'range discrepancy and wall normal discrepancy are different metrics'])
    os.environ.update(GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}', ROS_DOMAIN_ID=str(170+os.getpid()%20),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='', LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe')
    bridges = [v['topic']+'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' for v in views.values()]
    launch = ('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\nfrom launch_ros.actions import Node\n'
        'def generate_launch_description():\n    return LaunchDescription(['+
        f'ExecuteProcess(cmd={["gz", "sim", "-s", "-r", "-v", "4", str(RUN/"fixture.sdf")]!r}, output="screen"), '+
        f'Node(package="ros_gz_bridge", executable="parameter_bridge", arguments={bridges!r}, output="screen")])\n')
    (RUN/'fixture.launch.py').write_text(launch, encoding='utf-8')
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    node = rclpy.create_node('wall_raster_capture')
    frames = {name: [] for name in views}
    def receive(message, name):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if len(frames[name]) >= 3 or frames[name] and stamp <= frames[name][-1]['stamp']:
            return
        values = list(map(float, message.ranges))
        assert not any(math.isnan(v) or v == -math.inf for v in values)
        frames[name].append(dict(stamp=stamp, angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment), range_min=float(message.range_min),
            range_max=float(message.range_max), ranges=[None if v == math.inf else v for v in values]))
    subscriptions = [node.create_subscription(LaserScan, v['topic'], lambda m, n=n: receive(m, n), QoSProfile(depth=3)) for n, v in views.items()]
    process = None
    log_path = RUN/'launch.log'
    log = log_path.open('xb')
    try:
        process = subprocess.Popen(['ros2', 'launch', str(RUN/'fixture.launch.py')], cwd=RUN,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+90.
        while min(map(len, frames.values())) < 3 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('launch exited before capture')
            rclpy.spin_once(node, timeout_sec=.1)
        report['capture_complete'] = min(map(len, frames.values())) == 3
        report['captured_counts'] = {k: len(v) for k, v in frames.items()}
        report['comparison'] = compare(frames)
    except Exception as error:
        report['error'] = repr(error)
    finally:
        (RUN/'scan_frames.json').write_text(json.dumps(frames, allow_nan=False), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
        before = group_processes(process.pid) if process else {}
        report['owned_before_shutdown'] = before
        report['bridge_shutdown'] = stop_bridge_first(process, log_path.read_text(errors='replace'))
        report['gazebo_stop_service'] = stop_gazebo_service(process, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
        cleanup = stop_launch(process)
        log.close()
        report['shutdown_audit'] = audit_shutdown(log_path.read_text(errors='replace'), process.poll() if process else None, cleanup or None)
        report['residue_cleanup'] = cleanup_owned_residue(process.pid, before) if process else dict(actions=[], remaining={})
        report['source_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
        report['run_completed_cleanly'] = bool(report.get('capture_complete') and report['shutdown_audit']['clean']
            and report['bridge_shutdown']['verified'] and report['gazebo_stop_service']['verified']
            and not report['residue_cleanup']['actions'] and not report['residue_cleanup']['remaining']
            and report['source_unchanged'] and 'error' not in report)
        (RUN/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: report.get(k) for k in ('capture_complete', 'captured_counts', 'run_completed_cleanly', 'error')}))
    return 0 if report['run_completed_cleanly'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
