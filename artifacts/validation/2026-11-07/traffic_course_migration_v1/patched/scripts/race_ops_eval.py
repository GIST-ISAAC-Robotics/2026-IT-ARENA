#!/usr/bin/env python3
"""같은 경기 STOP/RESET/START 대표 흐름 평가. 격리 Gazebo 전용, 운영 도구가 아니다.

운영자 조작은 브라우저와 같은 HTTP API로만 한다. 차량 재배치와 정답 진행 확인은 이 평가기만
수행하며 제어기·어댑터에 정답·좌표·세이브포인트 번호·좌우 지정을 넘기지 않는다.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import zipfile

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'scripts'))
WORLD = REPO/'src/arena_gazebo/worlds/it_arena_official'
WORLD_NAME = 'it_arena_track'
POSE_TOPIC = f'/world/{WORLD_NAME}/dynamic_pose/info'
PROFILE = 'g1_open_10kmh_solo_gap05_arc_preview_bridge'
PORT = 18765
VEHICLE = 'eval_car'
SOURCES = ('race_session', 'local_pursuit', 'wall_follow', 'vision_process', 'lidar_motion_ros', 'lidar_safety',
           'race_ops_core', 'race_ops_node', 'race_ops_export', 'race_ops_page')
# 정지·재배치·상대 s좌표와 ID30·섬 통과 판정을 등록한 track. 다른 track은 재등록 전 실행하지 않는다.
REGISTERED_SCENARIOS = {'official': ('representative', 'across_marker', 'parked_recovery', 'browser_stop')}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Stop(RuntimeError):
    pass


class Ops:
    """브라우저 대역. 5 Hz로 상태를 읽고 명령마다 한 번만 보낸다."""
    def __init__(self, token):
        self.token, self.client = token, 'evalclient'
        self.state, self.state_wall = None, 0.
        self.polling, self.closed = True, False
        self.history = []
        self.number = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(f'http://127.0.0.1:{PORT}{path}', data=data, headers={
            'X-Arena-Token': self.token, 'X-Arena-Vehicle': VEHICLE, 'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=2.) as response:
            return response.read()

    def run(self):
        while not self.closed:
            if self.polling:
                try:
                    state = json.loads(self.request(f'/api/state?client={self.client}'))
                    self.state, self.state_wall = state, time.monotonic()
                    self.history.append(dict(wall=self.state_wall, state=state['state'], driving=state['driving'],
                                             race={k: state['race'].get(k) for k in ('latch', 'resume', 'last_resume')},
                                             binding=state['binding'], speeds=state['speeds']))
                except (OSError, ValueError, urllib.error.URLError):
                    pass
            time.sleep(.2)

    def body(self, op, confirm=None, view=None):
        self.number += 1
        return dict(op=op, request=f'eval-{self.number:03d}-{op[:6]}', client=self.client,
                    view=view if view is not None else None if op == 'stop' else self.state['binding'], confirm=confirm)

    def send(self, op, confirm=None, view=None):
        return json.loads(self.request('/api/command', self.body(op, confirm, view)))

    def send_without_reading_response(self, op):
        """응답 유실 재현: 요청을 보낸 뒤 응답을 읽지 않고 연결을 끊는다."""
        body = json.dumps(self.body(op)).encode()
        head = (f'POST /api/command HTTP/1.1\r\nHost: x\r\nX-Arena-Token: {self.token}\r\nX-Arena-Vehicle: {VEHICLE}\r\n'
                f'Content-Type: application/json\r\nContent-Length: {len(body)}\r\nConnection: close\r\n\r\n').encode()
        with socket.create_connection(('127.0.0.1', PORT), timeout=2.) as sock:
            sock.sendall(head+body)
        return self.number

    def command(self, request):
        return next((c for c in (self.state or {}).get('commands', []) if c['request'] == request), None)

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            state = self.state
            if state is not None and time.monotonic()-self.state_wall < 1.5 and predicate(state):
                return state
            time.sleep(.05)
        raise Stop(f'wait timeout: {label}; state={None if self.state is None else self.state["state"]}')


class Truth:
    """평가 전용 Gazebo 자세. 제어로 전달하지 않는다."""
    def __init__(self, output, log):
        self.rows, self.stream = [], (output/'ego_poses.jsonl').open('w', encoding='utf-8')
        self.process = subprocess.Popen(['gz', 'topic', '-e', '-t', POSE_TOPIC, '--json-output'], cwd=REPO, text=True,
                                        stdout=subprocess.PIPE, stderr=log, start_new_session=True)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        for line in self.process.stdout:
            try:
                data = json.loads(line)
            except ValueError:
                continue
            header = data.get('header', {}).get('stamp', {})
            when = float(header.get('sec', 0))+float(header.get('nsec', 0))*1e-9
            for pose in data.get('pose', []):
                if pose.get('name') != 'arena_car':
                    continue
                p, q = pose['position'], pose['orientation']
                yaw = math.atan2(2*(q.get('w', 1.)*q.get('z', 0.)+q.get('x', 0.)*q.get('y', 0.)),
                                 1-2*(q.get('y', 0.)**2+q.get('z', 0.)**2))
                row = (when, float(p.get('x', 0.)), float(p.get('y', 0.)), yaw, float(p.get('z', 0.)))
                if not self.rows or when > self.rows[-1][0]:
                    self.rows.append(row)
                    self.stream.write(json.dumps(row)+'\n')

    def latest(self):
        return self.rows[-1] if self.rows else None

    def speed(self):
        rows = self.rows[-30:]
        if len(rows) < 2 or rows[-1][0] <= rows[0][0]:
            return math.inf
        return math.dist(rows[0][1:3], rows[-1][1:3])/(rows[-1][0]-rows[0][0])

    def close(self):
        self.process.terminate()
        self.stream.flush()


class Course:
    def __init__(self, world=WORLD):
        route = np.genfromtxt(world/'centerline.csv', delimiter=',', names=True)
        self.xy = np.column_stack((route['x_m'], route['y_m']))
        self.s = route['s_m']
        self.branch = np.genfromtxt(world/'branch_1.csv', delimiter=',', skip_header=1, usecols=(0, 1))

    def nearest(self, x, y):
        index = int(np.argmin(np.sum((self.xy-(x, y))**2, axis=1)))
        return float(self.s[index]), float(np.linalg.norm(self.xy[index]-(x, y)))

    def in_branch(self, x, y):
        """지름길 중심선이 본선보다 확실히 가까우면 지름길에 들어간 것으로 본다. 평가 전용."""
        main = float(np.min(np.linalg.norm(self.xy-(x, y), axis=1)))
        return float(np.min(np.linalg.norm(self.branch-(x, y), axis=1)))+.05 < main

    def pose(self, s):
        x = float(np.interp(s, self.s, self.xy[:, 0]))
        y = float(np.interp(s, self.s, self.xy[:, 1]))
        x2 = float(np.interp(s+.15, self.s, self.xy[:, 0]))
        y2 = float(np.interp(s+.15, self.s, self.xy[:, 1]))
        return x, y, math.atan2(y2-y, x2-x)


def teleport(course, truth, s, log):
    """사람이 차를 옮겨 놓는 동작의 평가 전용 대역."""
    x, y, yaw = course.pose(s)
    z = truth.latest()[4]
    request = (f'name: "arena_car" position {{x: {x} y: {y} z: {z+.002}}} '
               f'orientation {{x: 0 y: 0 z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}')
    result = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose', '--reqtype', 'gz.msgs.Pose',
                             '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
                            stdout=subprocess.PIPE, stderr=log, text=True)
    if 'data: true' not in result.stdout:
        raise Stop('set_pose failed: '+result.stdout)
    return dict(s=s, x=x, y=y, yaw=yaw)


class Scenario:
    def __init__(self, args, output, ops, truth, course, log, processes):
        self.args, self.output, self.ops, self.truth, self.course = args, output, ops, truth, course
        self.scale = args.wall_scale
        self.log, self.processes = log, processes
        self.steps = []

    def note(self, name, **data):
        latest = self.truth.latest()
        state = self.ops.state or {}
        row = dict(step=name, wall_s=time.monotonic(), truth=latest, sim_s=(state.get('driving') or {}).get('sim_s'),
                   ui_state=state.get('state'), binding=state.get('binding'), side=(state.get('driving') or {}).get('side'),
                   **data)
        self.steps.append(row)
        print(json.dumps({k: v for k, v in row.items() if k not in ('truth', 'binding')}, ensure_ascii=False), flush=True)
        return row

    def press(self, op, **extra):
        entry = self.ops.send(op, **extra)
        self.note('press_'+op, request=entry['request'], stage=entry['stage'], adapter_reason=entry['adapter_reason'])
        return entry

    def wait(self, predicate, timeout, label):
        return self.ops.wait(predicate, timeout*self.scale, label)

    def hold(self, sim_seconds):
        """시뮬 시간 대기. 평가 시뮬의 실시간 비율은 1보다 훨씬 낮다."""
        end = self.truth.latest()[0]+sim_seconds
        deadline = time.monotonic()+sim_seconds*self.scale+5.
        while self.truth.latest()[0] < end and time.monotonic() < deadline:
            time.sleep(.05)

    def stage(self, entry, stages, timeout=6.):
        request = entry['request']
        self.wait(lambda s: (self.ops.command(request) or {}).get('stage') in stages, timeout, f'{entry["op"]} {stages}')
        return self.ops.command(request)

    def stopped(self):
        deadline = time.monotonic()+15.*self.scale
        while time.monotonic() < deadline:
            if self.truth.speed() < .02:
                return
            time.sleep(.05)
        raise Stop('car did not stop in truth')

    def stop_car(self):
        entry = self.press('stop')
        command = self.stage(entry, ('policy_accepted', 'policy_rejected', 'unknown_after_send'))
        self.wait(lambda s: s['mcu']['state'] == 'ESTOP_LATCHED', 3., 'MCU STOP state')
        self.stopped()
        self.note('stopped', policy=command['policy'], mcu=self.ops.command(entry['request'])['mcu'],
                  physical=command['physical_stop'], motor=self.ops.state['speeds']['motor_shaft_equiv_mps'],
                  stationary_ready=self.ops.state['mcu']['stationary_ready'])

    def until_s(self, target, timeout, label):
        deadline = time.monotonic()+timeout*self.scale
        while time.monotonic() < deadline:
            latest = self.truth.latest()
            s, _ = self.course.nearest(latest[1], latest[2])
            if target <= s < target+6.:
                return s
            time.sleep(.02)
        raise Stop(f'truth progress timeout: {label}')

    def resume(self, name, entry):
        """START 한 번의 결과를 끝까지 본다. 재전송하지 않는다."""
        begin = self.truth.latest()[0]
        self.wait(lambda s: s['race'].get('resume') or (s['race'].get('last_resume') or {}).get('request') == entry['request'],
                  4., 'pending seen')
        state = self.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 8., 'resume closed')
        self.note('resume_closed', scenario=name, last=state['race']['last_resume'], sim_seconds=self.truth.latest()[0]-begin,
                  route=state['driving']['route'])
        if state['race']['last_resume']['outcome'] != 'permitted':
            raise Stop('resume not permitted: '+str(state['race']['last_resume']))
        return state

    def recover(self, place_s, *, carry_s=None, reset_after=False, name='left'):
        if not reset_after:
            self.stage(self.press('recovery_reset'), ('policy_accepted',))
        if carry_s is not None:
            self.note('carry_to', **teleport(self.course, self.truth, carry_s, self.log))
            self.hold(1.)
            self.note('carry_seen', marker_ids=self.ops.state['driving']['marker_ids'])
        self.note('placed', **teleport(self.course, self.truth, place_s, self.log))
        self.hold(.5)
        self.note('after_place_hold', ui=self.ops.state['state'])
        if reset_after:
            self.stage(self.press('recovery_reset'), ('policy_accepted',))
        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed')

    def run_cli(self, *arguments):
        env = dict(os.environ)
        result = subprocess.run([sys.executable, '-B', str(REPO/'scripts/race_operator.py'), *arguments, '--timeout-s', '6'],
                                cwd=REPO, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return result.returncode, result.stdout.strip()[-600:]

    def first_start(self):
        ops = self.ops
        state = ops.wait(lambda s: s['binding'].get('session') and s['race'].get('latch') == 'process_start', 120., 'first status')
        self.note('process_start', latch=state['race']['latch'])
        self.press('resume_start')
        binding = ops.state['binding']
        code, text = self.run_cli('resume-start', '--session', binding['session'], '--race-epoch', str(binding['race_epoch']),
                                  '--intervention', str(binding['intervention']), '--send')
        self.note('first_start_cli_rejected', code=code, text=text)
        self.hold(.3)
        self.note('no_motion_before_prepare', truth_speed=self.truth.speed())
        entry = self.press('prepare_start')
        self.stage(entry, ('policy_accepted',))
        self.wait(lambda s: s['race']['policy_reason'] == 'wait_visual_start' and s['mcu']['state'] in ('ARMED', 'ACTIVE'),
                  20., 'armed waiting signal')
        self.note('armed_waiting_signal')
        self.wait(lambda s: s['race']['started'] is True, 30., 'visual start')
        self.note('visual_started', source=ops.state['race']['start_source'])
        self.until_s(self.args.first_stop_s, 40., 'first stop point')
        self.note('first_stop_point')

    def left_recovery(self):
        self.stop_car()
        place = self.args.place_left_s
        self.recover(place, carry_s=self.args.carry_s)
        side_before = self.ops.state['driving']['side']
        entry = self.press('resume_start')
        self.resume('left', entry)
        self.note('left_permitted', side_before=side_before)
        self.until_s(place+4., 25., 'left progress')
        self.note('left_progress')

    def right_section_tests(self):
        self.until_s(self.args.place_right_s-.2, 40., 'passed ID30')
        self.note('passed_id30_naturally', marker_side=self.ops.state['driving']['side_marker'])
        self.stop_car()
        self.recover(self.args.place_right_s, reset_after=True, name='right')
        # 준비 중 새 STOP: 그 START는 뒤늦게 실행되지 않는다.
        entry = self.press('resume_start')
        self.stage(entry, ('preparing', 'policy_accepted'))
        stop = self.press('stop')
        self.stage(stop, ('policy_accepted',))
        self.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 3., 'cancel by stop')
        self.note('cancelled_by_stop', last=self.ops.state['race']['last_resume'], duplicate=self.press('stop')['stage'])
        self.hold(1.)
        self.note('still_held_after_cancel', truth_speed=self.truth.speed())
        self.stage(self.press('recovery_reset'), ('policy_accepted',))
        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed again')
        # 준비 중 운영 연결 상실: 폴링을 멈춘 채 START 뒤 어댑터가 취소한다.
        self.ops.polling = False
        time.sleep(1.85)
        entry = self.press('resume_start')
        time.sleep(1.2)
        self.ops.polling = True
        self.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 4., 'link cancel')
        self.note('cancelled_by_link', last=self.ops.state['race']['last_resume'],
                  stage=self.ops.command(entry['request'])['stage'])
        self.hold(1.)
        self.note('no_late_start_after_reconnect', truth_speed=self.truth.speed())
        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed after link cancel')
        # 응답 유실 START: 재전송 없이 상태로 결과를 확인한다.
        self.ops.send_without_reading_response('resume_start')
        request = f'eval-{self.ops.number:03d}-resume'
        self.note('start_response_dropped', request=request)
        self.resume('right', dict(request=request))
        self.until_s(38.5, 30., 'right progress past island')
        self.note('right_progress')

    def restart_controller(self):
        pid = int(subprocess.check_output(['pgrep', '-f', '^/usr/bin/python3 .*lib/arena_autonomy/local_pursuit']).split()[0])
        command = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')[:-1]
        environ = dict(item.split('=', 1) for item in Path(f'/proc/{pid}/environ').read_bytes().decode(errors='replace').split('\0') if '=' in item)
        self.note('kill_controller', pid=pid)
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic()+20.
        while Path(f'/proc/{pid}').exists() and time.monotonic() < deadline:
            time.sleep(.1)
        if Path(f'/proc/{pid}').exists():
            os.kill(pid, signal.SIGKILL)
            time.sleep(.5)
        command = [*command, '-p', f'g1_trace_path:={self.output/"follow_decisions_restart.jsonl"}']
        process = subprocess.Popen(command, cwd=REPO, env=environ, stdin=subprocess.DEVNULL, stdout=self.log, stderr=self.log,
                                   start_new_session=True)
        self.processes.append(process)
        return process

    def restart_recovery(self):
        ops = self.ops
        self.stop_car()
        session = ops.state['binding']['session']
        self.restart_controller()
        self.note('placed', **teleport(self.course, self.truth, self.args.place_right_s, self.log))
        state = ops.wait(lambda s: s['binding'].get('session') not in (None, session) and s['race'].get('latch') == 'process_start',
                         180., 'new session')
        self.note('restarted', session=state['binding']['session'], started=state['race']['started'],
                  offer=state['restore_offer'])
        refused = self.press('resume_start')
        self.note('start_before_restore', reason=refused['adapter_reason'])
        unconfirmed = self.press('restore_race')
        self.note('restore_without_confirm', reason=unconfirmed['adapter_reason'], started=ops.state['race']['started'])
        entry = self.press('restore_race', confirm='SAME_RACE')
        self.stage(entry, ('policy_accepted',))
        state = self.wait(lambda s: s['race']['started'] is True and s['race']['recovery_ready'], 4., 'restored')
        self.note('restored', source=state['race']['start_source'], visual=state['race']['visual_start_fact'])
        self.hold(.5)
        self.note('hold_after_restore', truth_speed=self.truth.speed())
        entry = self.press('resume_start')
        self.resume('restart', entry)
        self.until_s(38.5, 30., 'restart progress past island')
        self.note('restart_progress')
        self.stop_car()

    def across_marker(self):
        """진단: 왼쪽 상태로 ID30을 건너 재배치. 통과 조건이 아니라 한계를 재는 장면이다."""
        self.stop_car()
        self.recover(self.args.place_right_s, name='across')
        self.resume('across', self.press('resume_start'))
        self.hold(self.args.across_watch_s)
        self.note('across_end', ui=self.ops.state['state'], reason=self.ops.state['driving']['reason'])
        self.press('stop')

    def parked_recovery(self):
        """현재 교통 제어로 정지 상대 뒤 재배치/재개/우회. 정답은 평가기에만 둔다."""
        self.stop_car()
        self.recover(1.0, name='parked')
        self.resume('parked', self.press('resume_start'))
        self.until_s(6.5, 35., 'recovered traffic progress')
        self.note('parked_progress')
        if self.args.browser_stop:
            self.note('await_browser_stop')
            try:
                self.ops.wait(lambda s: bool(s['race'].get('latch')) and s['mcu']['state'] == 'ESTOP_LATCHED',
                              180., 'actual browser STOP')
            except Stop:
                self.press('stop')
                raise
            self.stopped()
            self.hold(.3)
            self.note('browser_stop_observed', truth_speed=self.truth.speed(),
                      ui=self.ops.state['state'], mcu=self.ops.state['mcu']['state'],
                      commands=self.ops.state['commands'])
        else:
            self.stop_car()

    def browser_only(self):
        """화면 조작만 따로 확인한다. 별도 코스 완주를 반복하지 않는다."""
        self.ops.wait(lambda s: s['binding'].get('session') and s['race'].get('latch') == 'process_start',
                      120., 'browser initial state')
        self.note('await_browser_prepare')
        try:
            self.ops.wait(lambda s: s['race'].get('started') is True, 240., 'browser prepare and RGB signal')
            self.note('browser_visual_started', source=self.ops.state['race']['start_source'])
            self.until_s(.5, 15., 'browser short progress')
            self.note('await_browser_stop')
            self.ops.wait(lambda s: bool(s['race'].get('latch')) and s['mcu']['state']=='ESTOP_LATCHED',
                          180., 'actual browser STOP')
            self.stopped()
            self.hold(.3)
            self.note('browser_stop_observed', truth_speed=self.truth.speed(), ui=self.ops.state['state'],
                      mcu=self.ops.state['mcu']['state'], commands=self.ops.state['commands'])
            self.note('await_browser_snapshot')
            deadline = time.monotonic()+90.
            while not (self.output/'browser_capture_done').exists() and time.monotonic()<deadline:
                time.sleep(.1)
        finally:
            # UI 시험이 실패해도 STOP 수락/차체 정지는 따로 회수한다.
            self.stop_car()

    def run(self):
        if self.args.scenario == 'browser_stop':
            return self.browser_only()
        self.first_start()
        if self.args.scenario == 'across_marker':
            return self.across_marker()
        if self.args.scenario == 'parked_recovery':
            return self.parked_recovery()
        self.left_recovery()
        self.right_section_tests()
        self.restart_recovery()


def main():
    from validate_basic_autonomy import DEMO_TRACK, TRACK_WORLDS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=('representative', 'across_marker', 'parked_recovery', 'browser_stop'), default='representative')
    parser.add_argument('--browser-stop', action='store_true', help='parked_recovery 진행 뒤 실제 브라우저 STOP을 기다림')
    parser.add_argument('--first-stop-s', type=float, default=6.)
    parser.add_argument('--place-left-s', type=float, default=24.)
    parser.add_argument('--carry-s', type=float, default=31.6)
    parser.add_argument('--place-right-s', type=float, default=33.7)
    parser.add_argument('--across-watch-s', type=float, default=15.)
    parser.add_argument('--wall-limit-s', type=float, default=2400.)
    parser.add_argument('--wall-scale', type=float, default=14., help='시뮬 초당 벽시계 초 여유. 평가 환경 실시간 비율 약 0.09 기준')
    parser.add_argument('--track', choices=tuple(TRACK_WORLDS), default=DEMO_TRACK,
                        help='실행·평가 월드. 운영 시나리오를 등록한 track만 실행한다.')
    parser.add_argument('--local-sensor-config', type=Path, help='센서 프로필 명시 선택. 기본 track 외에는 필수')
    args = parser.parse_args()
    if args.browser_stop and args.scenario != 'parked_recovery':
        parser.error('--browser-stop requires parked_recovery')
    if args.track != DEMO_TRACK and args.local_sensor_config is None:
        parser.error('새 코스 운영 시험은 --local-sensor-config를 명시한다')
    if args.scenario not in REGISTERED_SCENARIOS.get(args.track, ()):
        parser.error(f'{args.track}의 {args.scenario} 운영 시나리오는 미등록이다. 옛 정지·재배치·상대 s좌표를 옮겨 쓰지 않고 재등록한다')
    world = TRACK_WORLDS[args.track]
    output = args.output.resolve()
    output.relative_to(REPO/'artifacts')
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(120+os.getpid() % 30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
                      GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}')
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    from lidar_shutdown import group_processes, stop_gazebo_service, stop_launch
    from validate_basic_autonomy import g1_solo_arguments, start_demo_process

    sources = {f'src/arena_autonomy/arena_autonomy/{name}.py': digest(REPO/f'src/arena_autonomy/arena_autonomy/{name}.py')
               for name in SOURCES}
    sources['scripts/race_ops_eval.py'] = digest(__file__)
    if args.track != DEMO_TRACK:
        for name in ('world.sdf', 'scene.json', 'centerline.csv', 'branch_1.csv'):
            sources[(world/name).relative_to(REPO).as_posix()] = digest(world/name)
    traffic = args.scenario == 'parked_recovery'
    if traffic:
        from validate_g1_traffic import actor_sdf, source_paths
        config_path = REPO/'config/tests/official_traffic_v1.json'
        for relative in source_paths(config_path):
            sources[relative] = digest(REPO/relative)
        # 새 운영 의존성과 현재 제어기 파일도 함께 보존한다.
        for directory in ('src/arena_autonomy/arena_autonomy', 'src/arena_vehicle_interface/arena_vehicle_interface'):
            for path in (REPO/directory).glob('*.py'):
                sources[path.relative_to(REPO).as_posix()] = digest(path)
        with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as snapshot:
            for relative in sources:
                snapshot.write(REPO/relative, relative)
    report = dict(scenario=args.scenario, started_wall=time.time(), source_sha256=sources, passed=False, steps=[])
    if args.track != DEMO_TRACK:
        report['track'] = args.track
    log_path = output/'simulation.log'
    log = log_path.open('w', encoding='utf-8')
    extra = g1_solo_arguments(True, 'corner_distance_candidate', 'per_ray_candidate', True, output, True, False)
    extra += [key+':=true' for key in ('g1_short_boundary_prefix_enabled', 'g1_parallel_hold_trial', 'g1_pass_release_trial',
                                       'g1_guard_curve_trial', 'g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial')]
    extra += ['partial_observation_policy:=bounded_local_v4', 'race_control_mode:=b3_candidate',
              'g1_standing_wall_reacquire:=true']
    if args.track != DEMO_TRACK:
        extra.append('track:='+args.track)
    if args.local_sensor_config is not None:
        sensor_path = args.local_sensor_config.resolve(strict=True)
        report['local_sensor_config'] = dict(path=str(sensor_path), sha256=digest(sensor_path))
        extra.append(f'local_sensor_config:={sensor_path}')
    if traffic:
        config = json.loads(config_path.read_text(encoding='utf-8'))
        actor_path = output/'parked_actor.sdf'
        actor_pose = Course(world).pose(2.1)
        actor_path.write_text(actor_sdf(config, actor_pose), encoding='utf-8')
        report['actor_evaluation_only'] = dict(s_m=2.1, pose=actor_pose, config=config['actor'])
        extra += [f'evaluation_actor_sdf:={actor_path}', 'evaluation_handoff_reserve:=paced',
                  'evaluation_handoff_yield_progress:=true', 'evaluation_handoff_rear_sweep:=true',
                  'evaluation_handoff_observed_motion:=true']
    report['speed_profile'] = 'g1_open_1p5' if traffic or args.scenario == 'browser_stop' else PROFILE
    launch = start_demo_process(log, report['speed_profile'], 10., True, 1.6, 'local_pursuit', False, 'sequential', 'both', 50., 'wsl_nvidia',
                                30., False, 'configured', str(output), 3, '', '', 'virtual_mcu', False, True, extra)
    processes = [launch]
    token = 'eval-token-'+os.urandom(6).hex()
    (output/'token.txt').write_text(token, encoding='utf-8')
    adapter = subprocess.Popen(['ros2', 'run', 'arena_autonomy', 'race_ops', '--ros-args', '-p', f'vehicle_id:={VEHICLE}',
                                '-p', f'port:={PORT}', '-p', f'token_path:={output/"token.txt"}',
                                '-p', f'record_path:={output/"race_record.json"}', '-p', f'log_path:={output/"ops_log.jsonl"}',
                                '-p', 'status_stale_s:=20.0', '-p', 'result_timeout_s:=30.0', '-p', 'mcu_timeout_s:=240.0'],
                               cwd=REPO, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    processes.append(adapter)
    topic_streams = []
    if traffic:
        for name, topic in (('actor_poses', '/model/eval_g1b_actor/pose'), ('contacts', '/eval/g1b_actor/contact')):
            stream = (output/(name+'.jsonl')).open('w', encoding='utf-8')
            topic_streams.append(stream)
            processes.append(subprocess.Popen(['gz', 'topic', '-e', '-t', topic, '--json-output'], cwd=REPO,
                             stdin=subprocess.DEVNULL, stdout=stream, stderr=log, start_new_session=True))
    time.sleep(3.)
    ops = Ops(token)
    truth = None
    scenario = None
    code = 1
    try:
        deadline = time.monotonic()+60.
        while truth is None or truth.latest() is None:
            truth = truth or Truth(output, log)
            if time.monotonic() > deadline:
                raise Stop('no truth pose')
            time.sleep(.2)
        course = Course(world)
        if traffic:
            report['contact_topic_info'] = subprocess.run(['gz', 'topic', '-i', '-t', '/eval/g1b_actor/contact'],
                capture_output=True, text=True, timeout=10.).stdout
        scenario = Scenario(args, output, ops, truth, course, log, processes)
        watchdog = threading.Timer(args.wall_limit_s, lambda: os.kill(os.getpid(), signal.SIGINT))
        watchdog.daemon = True
        watchdog.start()
        scenario.run()
        watchdog.cancel()
        report['completed'] = True
        code = 0
    except (Stop, KeyboardInterrupt, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
        print(report['error'], flush=True)
    finally:
        if scenario is not None:
            report['steps'] = scenario.steps
        try:
            report['final_state'] = ops.state
            export = ops.request('/api/export.html?seconds=3600')
            (output/'ops_export_live.html').write_bytes(export)
        except OSError as error:
            report['export_error'] = str(error)
        (output/'ops_history.json').write_text(json.dumps(ops.history, ensure_ascii=False), encoding='utf-8')
        ops.closed = True
        if truth is not None:
            truth.close()
        log.flush()
        report['gazebo_stop_service'] = stop_gazebo_service(launch, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
        report['auxiliary_cleanup'] = []
        for process in processes[1:]:
            # Every auxiliary is started in its own session. Signal that owned group,
            # not just the ros2-run parent (which waits for its unsignalled child).
            remaining = group_processes(process.pid)
            cleanup = dict(pgid=process.pid, members_before=remaining, forced=False)
            begin = time.monotonic()
            if process.poll() is None or remaining:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                deadline = begin+10.
                while time.monotonic() < deadline:
                    process.poll()
                    if not group_processes(process.pid):
                        break
                    time.sleep(.05)
                if group_processes(process.pid):
                    os.killpg(process.pid, signal.SIGKILL)
                    cleanup['forced'] = True
                process.wait(timeout=5.)
            cleanup.update(exit_code=process.poll(), elapsed_s=time.monotonic()-begin,
                           remaining_members=group_processes(process.pid))
            report['auxiliary_cleanup'].append(cleanup)
        report['launch_cleanup'] = stop_launch(launch)
        report['remaining_test_pids'] = sorted({pid for p in processes for pid in group_processes(p.pid)})
        report['exit_codes'] = [p.poll() for p in processes]
        for stream in topic_streams:
            stream.close()
        report['source_unchanged'] = all(digest(REPO/p) == d for p, d in sources.items())
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding='utf-8')
        log.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
