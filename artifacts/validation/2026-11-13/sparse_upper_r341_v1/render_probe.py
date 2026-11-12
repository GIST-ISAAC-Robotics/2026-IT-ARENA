"""작은 정지 Gazebo 월드에서 GPU lidar가 희소 visual을 실제로 보는지 확인."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from audit import A, ROOT, harness
from lidar_shutdown import group_processes, stop_gazebo_service, stop_launch


def main():
    out = A/'gpu_probe'
    out.mkdir(exist_ok=False)
    world = ET.fromstring('''<sdf version="1.9"><world name="r341">
      <physics name="p" type="ode"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <gravity>0 0 -9.8</gravity></world></sdf>''')
    w = world.find('world')
    placement = [('box', 0., 0.), ('thin_upper_blade', math.pi/2, 0.),
                 ('sparse_upper_posts', math.pi, math.pi), ('recessed_sides', -math.pi/2, -math.pi/2)]
    for index, (outline, azimuth, yaw) in enumerate(placement):
        cfg = harness.load_config(ROOT/'config/tests/official_traffic_v1.json')
        cfg['actor'].update(model_name=f'eval_sparse_{index}', visual_outline=outline, height_m=.22, bottom_m=.02)
        actor = ET.fromstring(harness.actor_sdf(cfg, (.8*math.cos(azimuth), .8*math.sin(azimuth), yaw)))
        model = actor.find('model')
        for plugin in list(model.findall('plugin')):
            model.remove(plugin)
        for sensor in list(model.findall('link/sensor')):
            model.find('link').remove(sensor)
        w.append(model)
    sensor_model = ET.SubElement(w, 'model', name='probe_lidar')
    ET.SubElement(sensor_model, 'static').text = 'true'
    ET.SubElement(sensor_model, 'pose').text = '0 0 .1346 0 0 0'
    link = ET.SubElement(sensor_model, 'link', name='sensor')
    for count in (512, 3600):
        link.append(ET.fromstring(f'''<sensor name="scan{count}" type="gpu_lidar"><always_on>true</always_on>
          <update_rate>10</update_rate><topic>/r341/scan{count}</topic><lidar><scan><horizontal>
          <samples>{count}</samples><resolution>1</resolution><min_angle>{-math.pi}</min_angle><max_angle>{math.pi}</max_angle>
          </horizontal></scan><range><min>.05</min><max>12</max><resolution>.001</resolution></range></lidar></sensor>'''))
    ET.indent(world)
    sdf = out/'world.sdf'
    ET.ElementTree(world).write(sdf, encoding='utf-8', xml_declaration=True)
    launch = out/'probe.launch.py'
    launch.write_text('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\ndef generate_launch_description():\n'
        f'    return LaunchDescription([ExecuteProcess(cmd=["gz","sim","-s","-r","--headless-rendering",{str(sdf)!r}], output="screen")])\n')
    os.environ['GZ_PARTITION'] = f'arena_autonomy_test_{os.getpid()}'
    log_path = out/'launch.log'
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', str(launch)], stdout=log, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, start_new_session=True)
    report = dict(scope='stationary GPU visual probe; ideal 512 and 3600 rays, not native moving C1 or driving',
                  actors=placement, samples={})
    try:
        for count in (512, 3600):
            command = ['gz', 'topic', '-e', '-n', '1', '--json-output', '-t', f'/r341/scan{count}']
            received = subprocess.run(command, capture_output=True, text=True, timeout=45)
            (out/f'scan{count}.jsonl').write_text(received.stdout)
            (out/f'scan{count}.stderr').write_text(received.stderr)
            data = next(json.loads(line) for line in received.stdout.splitlines() if line.startswith('{'))
            values = [float(value) for value in data['ranges']]
            angles = [float(data['angle_min'])+i*float(data['angle_step']) for i in range(len(values))]
            by_actor = {}
            for outline, azimuth, _ in placement:
                hits = [(a, r) for a, r in zip(angles, values) if math.isfinite(r) and .65 < r < .95
                        and abs(math.atan2(math.sin(a-azimuth), math.cos(a-azimuth))) < .2]
                by_actor[outline] = len(hits)
            report['samples'][count] = dict(ranges=len(values), hits=by_actor)
    except Exception as error:
        report['error'] = repr(error)
    finally:
        report['server_stop'] = stop_gazebo_service(process, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
        report['launch_cleanup'] = stop_launch(process)
        log.close()
        report['remaining_pids'] = list(group_processes(process.pid))
        report['return_code'] = process.returncode
        report['hashes'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in (Path(__file__), A/'audit.py', sdf, ROOT/'scripts/validate_g1_traffic.py')}
        (out/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        print(json.dumps(report, indent=2, ensure_ascii=False))
    return int('error' in report or bool(report['remaining_pids']) or bool(report['launch_cleanup']))


if __name__ == '__main__':
    raise SystemExit(main())
