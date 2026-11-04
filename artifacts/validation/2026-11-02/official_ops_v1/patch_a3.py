"""a2 실패 반영: ARM 뒤 재DISARM 방지, 벽시계 한도 매개변수화, 평가기 시뮬 시간 대기. 치환 실패 시 중단."""
from pathlib import Path


def patch(path, pairs):
    target = Path(path)
    raw = target.read_bytes()
    crlf = raw.count(b'\r\n') == raw.count(b'\n') and b'\r\n' in raw
    text = raw.replace(b'\r\n', b'\n').decode('utf-8')
    for old, new in pairs:
        assert text.count(old) >= 1, old
        text = text.replace(old, new)
    data = text.encode('utf-8')
    target.write_bytes(data.replace(b'\n', b'\r\n') if crlf else data)


patch('src/arena_autonomy/arena_autonomy/race_ops_node.py', [
("""MCU_CALL_TIMEOUT_S = 2.
PREPARE_TIMEOUT_S = 30.
""", """MCU_CALL_TIMEOUT_S = 2.
"""),
("""                        record_path='', log_path='', status_stale_s=1., client_timeout_s=2., record_max_age_s=7200.,
                        log_max_bytes=64*1024*1024)
""", """                        record_path='', log_path='', status_stale_s=1., client_timeout_s=2., record_max_age_s=7200.,
                        result_timeout_s=3., mcu_timeout_s=30., log_max_bytes=64*1024*1024)
"""),
("""        self.record_path = str(values['record_path'])
""", """        self.record_path = str(values['record_path'])
        self.result_timeout_s, self.mcu_timeout_s = float(values['result_timeout_s']), float(values['mcu_timeout_s'])
"""),
("""                if self.rearm is not None and self.rearm.get('future') is future:
                    self.rearm['future'] = None
            elif wall >= deadline:
""", """                if self.rearm is not None and self.rearm.get('future') is future:
                    self.rearm['future'] = None
                    # 상태 토픽은 ARM 응답보다 늦다. 늦은 상태로 방금 올린 ARM을 다시 내리지 않는다.
                    self.rearm['armed'] = self.rearm.get('armed') or (name == 'arm' and result.success)
            elif wall >= deadline:
"""),
("""        if job['future'] is not None or wall < job['next'] or not mcu:
            return
""", """        if job['future'] is not None or job.get('armed') or wall < job['next'] or not mcu:
            return
"""),
("""            self.rearm = dict(entry=entry, deadline=time.monotonic()+PREPARE_TIMEOUT_S, next=0., future=None)
""", """            self.rearm = dict(entry=entry, deadline=time.monotonic()+self.mcu_timeout_s, next=0., future=None)
"""),
("""                self.rearm = dict(entry=entry, deadline=wall+MCU_CALL_TIMEOUT_S*2, next=0., future=None)
""", """                self.rearm = dict(entry=entry, next=0., future=None)
"""),
("""        self.core.expire(wall)
""", """        self.core.expire(wall, self.result_timeout_s)
"""),
])

patch('src/arena_autonomy/arena_autonomy/local_pursuit.py', [
("""                mcu=McuObserver(wall_timeout_s=self.settings['sensor_wall_timeout_s']), now_s=self.now_s())
""", """                mcu=McuObserver(wall_timeout_s=self.settings['sensor_wall_timeout_s']), now_s=self.now_s(),
                resume_wall_timeout_s=3*self.settings['sensor_wall_timeout_s'])
"""),
])

patch('scripts/race_ops_eval.py', [
("""    def __init__(self, args, output, ops, truth, course, log, processes):
        self.args, self.output, self.ops, self.truth, self.course = args, output, ops, truth, course
""", """    def __init__(self, args, output, ops, truth, course, log, processes):
        self.args, self.output, self.ops, self.truth, self.course = args, output, ops, truth, course
        self.scale = args.wall_scale
"""),
("""    def stage(self, entry, stages, timeout=6.):
        request = entry['request']
        self.ops.wait(lambda s: (self.ops.command(request) or {}).get('stage') in stages, timeout, f'{entry["op"]} {stages}')
        return self.ops.command(request)
""", """    def wait(self, predicate, timeout, label):
        return self.ops.wait(predicate, timeout*self.scale, label)

    def hold(self, sim_seconds):
        \"\"\"시뮬 시간 대기. 평가 시뮬의 실시간 비율은 1보다 훨씬 낮다.\"\"\"
        end = self.truth.latest()[0]+sim_seconds
        deadline = time.monotonic()+sim_seconds*self.scale+5.
        while self.truth.latest()[0] < end and time.monotonic() < deadline:
            time.sleep(.05)

    def stage(self, entry, stages, timeout=6.):
        request = entry['request']
        self.wait(lambda s: (self.ops.command(request) or {}).get('stage') in stages, timeout, f'{entry["op"]} {stages}')
        return self.ops.command(request)
"""),
("""        deadline = time.monotonic()+15.
        while time.monotonic() < deadline:
            if self.truth.speed() < .02:
""", """        deadline = time.monotonic()+15.*self.scale
        while time.monotonic() < deadline:
            if self.truth.speed() < .02:
"""),
("""        self.ops.wait(lambda s: s['mcu']['state'] == 'ESTOP_LATCHED', 3., 'MCU STOP state')
""", """        self.wait(lambda s: s['mcu']['state'] == 'ESTOP_LATCHED', 3., 'MCU STOP state')
"""),
("""        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            latest = self.truth.latest()
""", """        deadline = time.monotonic()+timeout*self.scale
        while time.monotonic() < deadline:
            latest = self.truth.latest()
"""),
("""        wall = time.monotonic()
        self.ops.wait(lambda s: s['race'].get('resume') or (s['race'].get('last_resume') or {}).get('request') == entry['request'],
                      4., 'pending seen')
        state = self.ops.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 15., 'resume closed')
        self.note('resume_closed', scenario=name, last=state['race']['last_resume'], seconds=time.monotonic()-wall,
                  route=state['driving']['route'])
""", """        begin = self.truth.latest()[0]
        self.wait(lambda s: s['race'].get('resume') or (s['race'].get('last_resume') or {}).get('request') == entry['request'],
                  4., 'pending seen')
        state = self.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 8., 'resume closed')
        self.note('resume_closed', scenario=name, last=state['race']['last_resume'], sim_seconds=self.truth.latest()[0]-begin,
                  route=state['driving']['route'])
"""),
("""            self.note('carry_to', **teleport(self.course, self.truth, carry_s, self.log))
            time.sleep(1.5)
            self.note('carry_seen', marker_ids=self.ops.state['driving']['marker_ids'])
""", """            self.note('carry_to', **teleport(self.course, self.truth, carry_s, self.log))
            self.hold(1.)
            self.note('carry_seen', marker_ids=self.ops.state['driving']['marker_ids'])
"""),
("""        time.sleep(.6)
        self.note('after_place_hold', ui=self.ops.state['state'])
""", """        self.hold(.5)
        self.note('after_place_hold', ui=self.ops.state['state'])
"""),
("""        self.ops.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed')
""", """        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed')
"""),
("""        state = ops.wait(lambda s: s['binding'].get('session') and s['race'].get('latch') == 'process_start', 90., 'first status')
""", """        state = ops.wait(lambda s: s['binding'].get('session') and s['race'].get('latch') == 'process_start', 120., 'first status')
"""),
("""        time.sleep(.5)
        self.note('no_motion_before_prepare', truth_speed=self.truth.speed())
""", """        self.hold(.3)
        self.note('no_motion_before_prepare', truth_speed=self.truth.speed())
"""),
("""        ops.wait(lambda s: s['race']['policy_reason'] == 'wait_visual_start', 40., 'armed waiting signal')
        self.note('armed_waiting_signal')
        ops.wait(lambda s: s['race']['started'] is True, 60., 'visual start')
""", """        self.wait(lambda s: s['race']['policy_reason'] == 'wait_visual_start' and s['mcu']['state'] in ('ARMED', 'ACTIVE'),
                  20., 'armed waiting signal')
        self.note('armed_waiting_signal')
        self.wait(lambda s: s['race']['started'] is True, 30., 'visual start')
"""),
("""        self.ops.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 3., 'cancel by stop')
""", """        self.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 3., 'cancel by stop')
"""),
("""        time.sleep(1.5)
        self.note('still_held_after_cancel', truth_speed=self.truth.speed())
""", """        self.hold(1.)
        self.note('still_held_after_cancel', truth_speed=self.truth.speed())
"""),
("""        self.ops.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed again')
""", """        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed again')
"""),
("""        self.ops.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 4., 'link cancel')
""", """        self.wait(lambda s: (s['race'].get('last_resume') or {}).get('request') == entry['request'], 4., 'link cancel')
"""),
("""        time.sleep(1.5)
        self.note('no_late_start_after_reconnect', truth_speed=self.truth.speed())
        self.ops.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed after link cancel')
""", """        self.hold(1.)
        self.note('no_late_start_after_reconnect', truth_speed=self.truth.speed())
        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'resume allowed after link cancel')
"""),
("""        state = ops.wait(lambda s: s['binding'].get('session') not in (None, session) and s['race'].get('latch') == 'process_start',
                         60., 'new session')
""", """        state = ops.wait(lambda s: s['binding'].get('session') not in (None, session) and s['race'].get('latch') == 'process_start',
                         180., 'new session')
"""),
("""        state = ops.wait(lambda s: s['race']['started'] is True and s['race']['recovery_ready'], 4., 'restored')
""", """        state = self.wait(lambda s: s['race']['started'] is True and s['race']['recovery_ready'], 4., 'restored')
"""),
("""        time.sleep(.6)
        self.note('hold_after_restore', truth_speed=self.truth.speed())
""", """        self.hold(.5)
        self.note('hold_after_restore', truth_speed=self.truth.speed())
"""),
("""        deadline = time.monotonic()+self.args.across_watch_s
        while time.monotonic() < deadline:
            time.sleep(.2)
""", """        self.hold(self.args.across_watch_s)
"""),
("""    parser.add_argument('--wall-limit-s', type=float, default=600.)
""", """    parser.add_argument('--wall-limit-s', type=float, default=2400.)
    parser.add_argument('--wall-scale', type=float, default=14., help='시뮬 초당 벽시계 초 여유. 평가 환경 실시간 비율 약 0.09 기준')
"""),
("""                                '-p', f'record_path:={output/"race_record.json"}', '-p', f'log_path:={output/"ops_log.jsonl"}'],
""", """                                '-p', f'record_path:={output/"race_record.json"}', '-p', f'log_path:={output/"ops_log.jsonl"}',
                                '-p', 'status_stale_s:=20.0', '-p', 'result_timeout_s:=30.0', '-p', 'mcu_timeout_s:=240.0'],
"""),
])

patch('tests/test_race_ops.py', [
("""    core.expire(102.)
    assert entry['stage'] == 'sent'
    core.expire(103.3)
""", """    core.expire(102., 3.)
    assert entry['stage'] == 'sent'
    core.expire(103.3, 3.)
"""),
])
