#!/usr/bin/env python3
"""G1-b 평가 전제 확인: set_pose로 옮긴 정적 actor의 충돌체·접촉 센서가 실제로 따라가는가.

자율주행 스택과 무관한 작은 격리 Gazebo 실험이다. 결과는 평가용 actor 설계(충돌체를
둘지, 접촉 확증을 쓸 수 있는지)의 근거로만 사용한다. 차량 안전 시험이 아니다.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

WORLD = 'actor_probe'
SDF = f"""<?xml version='1.0'?>
<sdf version='1.9'><world name='{WORLD}'>
  <physics name='p' type='ode'><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
  <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>
  <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
  <plugin filename='gz-sim-contact-system' name='gz::sim::systems::Contact'/>
  <gravity>0 0 -9.8</gravity>
  <model name='ground'><static>true</static><link name='l'><collision name='c'><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision></link></model>
  <model name='eval_actor'><static>true</static><pose>0 0 0.06 0 0 0</pose>
    <link name='body'>
      <collision name='collision'><geometry><box><size>0.20 0.15 0.12</size></box></geometry></collision>
      <visual name='visual'><geometry><box><size>0.20 0.15 0.12</size></box></geometry></visual>
      <sensor name='contact' type='contact'><always_on>true</always_on><update_rate>200</update_rate>
        <contact><collision>collision</collision><topic>/probe/actor_contact</topic></contact></sensor>
    </link></model>
  <model name='mover'><pose>1.0 0 0.05 0 0 0</pose>
    <link name='body'><inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <collision name='c'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>
  <model name='dropper'><pose>0 0.6 0.30 0 0 0</pose>
    <link name='body'><inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <collision name='c'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>
</world></sdf>
"""


def poses(env):
    out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '--json-output', '-t', f'/world/{WORLD}/dynamic_pose/info'],
                         env=env, capture_output=True, text=True, timeout=20).stdout
    data = json.loads(out.splitlines()[0])
    return {p['name']: {k: p['position'].get(k, 0.) for k in 'xyz'} for p in data.get('pose', [])}


def set_pose(env, name, x, y, z):
    request = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
    out = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'gz.msgs.Pose',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
                         env=env, capture_output=True, text=True, timeout=20)
    return out.stdout.strip()


def contacts(env, seconds):
    reader = subprocess.Popen(['gz', 'topic', '-e', '--json-output', '-t', '/probe/actor_contact'], env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
    time.sleep(seconds)
    os.killpg(reader.pid, signal.SIGINT)
    try:
        text = reader.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
        os.killpg(reader.pid, signal.SIGKILL)
        text = reader.communicate()[0]
    names = set()
    count = 0
    for line in text.splitlines():
        try:
            for contact in json.loads(line).get('contact', []):
                count += 1
                names.add((contact['collision1']['name'], contact['collision2']['name']))
        except (ValueError, KeyError):
            continue
    return {'messages_with_contacts': count, 'pairs': sorted(map(list, names))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    world = args.output/'probe_world.sdf'
    world.write_text(SDF, encoding='utf-8')
    env = dict(os.environ, GZ_PARTITION=f'actor_probe_{os.getpid()}')
    log = (args.output/'gz.log').open('w')
    server = subprocess.Popen(['gz', 'sim', '-r', '-s', '-v', '2', str(world)], env=env, stdout=log, stderr=log,
                              start_new_session=True)
    report = {'scope': 'isolated static-actor physics/contact probe; not vehicle safety evidence'}
    try:
        time.sleep(6)
        report['initial'] = poses(env)
        report['contacts_before_move'] = contacts(env, 1.5)
        report['set_pose_overlap_mover'] = set_pose(env, 'eval_actor', 1.0, 0., .06)
        time.sleep(1.5)
        report['after_move'] = poses(env)
        report['contacts_after_move'] = contacts(env, 1.5)
        # 옛 위치에 충돌체가 남는지: 원래 자리 위로 낙하시킨 상자의 높이를 본다.
        report['set_pose_dropper'] = set_pose(env, 'dropper', 0., 0., .30)
        time.sleep(2.0)
        report['after_drop_on_old_location'] = poses(env)
        mover0, mover1 = report['initial']['mover'], report['after_move']['mover']
        report['mover_displacement_m'] = ((mover1['x']-mover0['x'])**2+(mover1['y']-mover0['y'])**2
                                          + (mover1['z']-mover0['z'])**2)**.5
        report['dropper_rest_z_m'] = report['after_drop_on_old_location']['dropper']['z']
        report['interpretation'] = {
            'moved_collision_acts': report['mover_displacement_m'] > .02,
            'moved_contact_reported': report['contacts_after_move']['messages_with_contacts'] > 0,
            'ghost_at_old_location': report['dropper_rest_z_m'] > .09,
        }
    finally:
        os.killpg(server.pid, signal.SIGINT)
        try:
            server.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait()
        log.close()
        report['server_returncode'] = server.returncode
        (args.output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
