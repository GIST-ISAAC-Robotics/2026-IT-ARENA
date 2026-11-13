"""온보드 경기 운영 어댑터: 제한 HTTP API ↔ 기존 /race/command·MCU 서비스.

제어 콜백과 다른 프로세스다. 기본 패널은 정지 중 속도 상한 설정을 제공한다.
명시적인 별도 수동 구성만 기존 jog의 /drive 발행을 사용하며 /drive/safe에 직접 발행하지 않는다.
ROS 호출은 단일 실행기 타이머가 수행한다. 파일 기록과 선택형 JPEG 처리는 별도 스레드다.
"""
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arena_autonomy import race_ops_export
from arena_autonomy.race_ops_core import ARMED_MCU, LATCHED_MCU, OpsCore, token
from arena_autonomy.race_ops_page import PAGE
from arena_autonomy.race_ops_manual import ManualPanel, graph_reason
from arena_autonomy.race_ops_preview import Preview
from arena_autonomy.development_jog import fresh_status, valid_mcu, mcu_identity
from arena_vehicle_interface.node_lifecycle import run_node
from arena_vehicle_interface.queued_log import QueuedLog

MAX_BODY = 4096
MCU_RETRY_S = .1
MCU_CALL_TIMEOUT_S = 2.
LINK_PERIOD_S = .2
RECORD_PERIOD_S = 5.


def load_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def write_atomic(path, text):
    target = Path(path)
    temporary = target.with_suffix(target.suffix+'.tmp')
    temporary.write_text(text, encoding='utf-8')
    os.replace(temporary, target)


class Handler(BaseHTTPRequestHandler):
    server_version = 'race_ops'
    protocol_version = 'HTTP/1.1'

    def log_message(self, *_):
        pass

    def respond(self, code, body, kind='application/json; charset=utf-8', **headers):
        data = body if isinstance(body, bytes) else body.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        for key, value in headers.items():
            self.send_header(key.replace('_', '-'), value)
        self.end_headers()
        self.wfile.write(data)

    def role(self):
        """토큰과 차량 이름이 모두 맞아야 한다. 잘못된 차량을 조작하지 않게 하는 실수 방지이자 최소 접근 제한이다."""
        ops = self.server.ops
        given = self.headers.get('X-Arena-Token', '')
        if self.headers.get('X-Arena-Vehicle', '') != ops.vehicle_id:
            self.respond(409, json.dumps(dict(error='vehicle_mismatch')))
            return None
        if hmac.compare_digest(given.encode(), ops.token.encode()):
            return 'operator'
        if ops.view_token and hmac.compare_digest(given.encode(), ops.view_token.encode()):
            return 'view'
        self.respond(401, json.dumps(dict(error='unauthorized')))
        return None

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == '/':
            return self.respond(200, PAGE, 'text/html; charset=utf-8')
        if url.path == '/favicon.ico':
            return self.respond(204, b'')
        role = self.role()
        if role is None:
            return None
        query = parse_qs(url.query)
        wall, ops = time.monotonic(), self.server.ops
        if url.path == '/api/state':
            client = query.get('client', [''])[0]
            if token(client) and role == 'operator':
                ops.core.client_seen(client, wall)
            if token(client):ops.preview.keepalive(client)
            state = ops.core.view(wall, time.time())
            state['manual'] = ops.manual.snapshot()
            state['preview'] = ops.preview.snapshot()
            return self.respond(200, json.dumps(state, ensure_ascii=False, allow_nan=False), X_Arena_Role=role)
        if url.path == '/api/preview.jpg':
            data = ops.preview.jpeg()
            if data is None:return self.respond(404, json.dumps(dict(error='preview_off_stale_or_pending')))
            return self.respond(200, data, 'image/jpeg')
        if url.path in ('/api/export.csv', '/api/export.html'):
            seconds = float(query.get('seconds', ['120'])[0])
            table = race_ops_export.merge(ops.core.export_rows(max(1., min(seconds, 3600.)), wall))
            if url.path.endswith('.csv'):
                return self.respond(200, race_ops_export.render_csv(table), 'text/csv; charset=utf-8')
            return self.respond(200, race_ops_export.render_html(table, f'{ops.vehicle_id} 사건 시간축'),
                                'text/html; charset=utf-8')
        return self.respond(404, json.dumps(dict(error='not_found')))

    def do_POST(self):
        role = self.role()
        if role is None:
            return None
        path = urlsplit(self.path).path
        if path not in ('/api/command', '/api/manual', '/api/preview'):
            return self.respond(404, json.dumps(dict(error='not_found')))
        if role != 'operator' and path != '/api/preview':
            return self.respond(403, json.dumps(dict(error='read_only')))
        size = int(self.headers.get('Content-Length', '0') or 0)
        if not 0 < size <= MAX_BODY:
            return self.respond(413, json.dumps(dict(error='invalid_body_size')))
        try:
            body = json.loads(self.rfile.read(size))
        except ValueError:
            return self.respond(400, json.dumps(dict(error='invalid_json')))
        if path == '/api/preview':
            if type(body) is not dict or not token(body.get('client')):
                return self.respond(400, json.dumps(dict(error='invalid_preview_client')))
            try:
                result = self.server.ops.preview.configure(body.get('config'), body['client'])
            except ValueError as error:
                return self.respond(400, json.dumps(dict(error=str(error))))
            return self.respond(200, json.dumps(result, ensure_ascii=False, allow_nan=False))
        if (type(body) is not dict or not token(body.get('request')) or type(body.get('op')) is not str
                or not token(body.get('client'))):
            return self.respond(400, json.dumps(dict(error='schema')))
        if path == '/api/manual':
            entry = self.server.ops.manual.submit(body['op'], body['request'], body['client'],
                body.get('view'), body.get('value'), time.monotonic())
        else:
            entry = self.server.ops.core.submit(body['op'], body['request'], body.get('view'), body['client'],
                                                body.get('confirm'), time.monotonic(), value=body.get('value'))
        return self.respond(200, json.dumps(entry, ensure_ascii=False, allow_nan=False, default=str))


class RaceOps(Node):
    def __init__(self):
        super().__init__('race_ops')
        declared = dict(vehicle_id='arena', host='127.0.0.1', port=8765, token_path='', view_token_path='',
                        record_path='', log_path='', status_stale_s=1., client_timeout_s=2., record_max_age_s=7200.,
                        result_timeout_s=3., mcu_timeout_s=30., log_max_bytes=64*1024*1024,
                        operation_mode='autonomous', manual_maximum_mps=1.5, manual_steering_limit_rad=.2,
                        manual_output_root='build/panel_manual')
        values = {key: self.declare_parameter(key, value).value for key, value in declared.items()}
        self.vehicle_id = str(values['vehicle_id'])
        if not token(self.vehicle_id):
            raise ValueError('vehicle_id must be a short token')
        self.token = self.load_token(str(values['token_path']), create=True)
        self.view_token = self.load_token(str(values['view_token_path']), create=False)
        self.record_path = str(values['record_path'])
        self.result_timeout_s, self.mcu_timeout_s = float(values['result_timeout_s']), float(values['mcu_timeout_s'])
        self.core = OpsCore(self.vehicle_id, status_stale_s=float(values['status_stale_s']),
                            client_timeout_s=float(values['client_timeout_s']),
                            record_max_age_s=float(values['record_max_age_s']))
        if self.record_path:
            self.core.set_record(load_json(self.record_path))
        self.log_path, self.log_max_bytes = str(values['log_path']), int(values['log_max_bytes'])
        self.log_bytes = 0
        self.log_stream = open(self.log_path, 'a', encoding='utf-8') if self.log_path else None
        self.log = QueuedLog(self.write_log, capacity=2048)
        self.record_writer = QueuedLog(self.write_record, capacity=4)
        self.command_publisher = self.create_publisher(String, '/race/command', 10)
        self.link_publisher = self.create_publisher(String, '/race/operator_link', 10)
        self.create_subscription(String, '/autonomy/status', self.on_status, 20)
        self.create_subscription(String, '/race/command_result', self.on_result, 20)
        self.create_subscription(String, '/actuation/status', self.on_mcu, 20)
        self.create_subscription(String, '/safety/status', self.on_safety, 20)
        self.create_subscription(AckermannDriveStamped, '/drive', lambda m: self.on_drive('/drive', m), 5)
        self.create_subscription(AckermannDriveStamped, '/drive/safe', lambda m: self.on_drive('/drive/safe', m), 5)
        self.mcu_clients = {name: self.create_client(Trigger, '/actuation/'+name)
                         for name in ('stop', 'clear_stop', 'arm', 'disarm')}
        self.calls = []       # (entry, step, future, deadline)
        self.start = None     # 대기 중인 START 묶음
        self.rearm = None     # 진행 중인 CLEAR/ARM 묶음
        self.last_link = self.last_record = 0.
        self.last_written = None
        self.manual = ManualPanel(str(values['operation_mode']), float(values['manual_maximum_mps']),
            float(values['manual_steering_limit_rad']), str(values['manual_output_root']))
        self.manual_prepare = None
        self.preview = Preview()
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.server = ThreadingHTTPServer((str(values['host']), int(values['port'])), Handler)
        self.server.daemon_threads = True
        self.server.ops = self
        self.thread = threading.Thread(target=self.server.serve_forever, name='race_ops_http', daemon=True)
        self.thread.start()
        self.get_logger().info(f"경기 운영 어댑터: {values['host']}:{self.server.server_address[1]} 차량 {self.vehicle_id}")

    def load_token(self, path, *, create):
        if not path:
            if create:
                raise ValueError('token_path is required')
            return ''
        target = Path(path)
        if target.exists():
            value = target.read_text(encoding='utf-8').strip()
        elif create:
            value = secrets.token_urlsafe(18)
            target.touch(mode=0o600)
            target.write_text(value, encoding='utf-8')
        else:
            value = ''
        if create and len(value) < 8:
            raise ValueError('operator token too short')
        return value

    def write_log(self, text):
        if self.log_stream is None or self.log_bytes > self.log_max_bytes:
            return
        self.log_stream.write(text+'\n')
        self.log_stream.flush()
        self.log_bytes += len(text)+1

    def write_record(self, text):
        write_atomic(self.record_path, text)

    # ROS 입력 ------------------------------------------------------------
    def on_status(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        if type(status) is dict and type(status.get('race')) is dict:
            self.core.update_status(status, time.monotonic())

    def on_mcu(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        if type(status) is dict:
            self.core.update_mcu(status, time.monotonic())

    def on_safety(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        if type(status) is dict:
            self.core.update_safety(status, time.monotonic())

    def on_drive(self, topic, message):
        self.core.update_drive(topic, float(message.drive.speed), float(message.drive.steering_angle), time.monotonic())

    def on_result(self, message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if type(payload) is not dict:
            return
        entry = self.core.result(payload, time.monotonic())
        if entry is None or payload.get('accepted') is not True:
            if self.start is not None and entry is self.start['entry']:
                self.start = None
            return
        if entry['op'] == 'resume_start':
            view = entry['view']
            if (payload.get('op') != 'resume_start' or payload.get('reason') != 'preparing_not_motion_permission'
                    or any(payload.get(k) != view[k] for k in ('session', 'race_epoch', 'intervention'))):
                self.core.mark(entry, time.monotonic(), stage='unknown_after_send', outcome='start_ack_mismatch')
                return
            wall = time.monotonic()
            self.start = dict(entry=entry, client=entry['client'], session=view['session'],
                              status_deadline=wall+self.result_timeout_s)
            self.core.mark(entry, wall, stage='preparing')
        elif entry['op'] == 'prepare_start':
            self.rearm = dict(entry=entry, deadline=time.monotonic()+self.mcu_timeout_s, next=0., future=None)

    # 명령 실행 -----------------------------------------------------------
    def race(self):
        return (self.core.status or {}).get('race') or {}

    def publish(self, payload):
        self.command_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))

    def bound(self, entry, op):
        view = entry['view']
        return dict(v=1, request=entry['request'], op=op, session=view['session'],
                    race_epoch=view['race_epoch'], intervention=view['intervention'])

    def execute(self, entry):
        op, wall = entry['op'], time.monotonic()
        if self.manual.mode == 'manual' and op != 'stop':
            return self.core.mark(entry, wall, stage='rejected_by_adapter', adapter_reason='manual_configuration')
        if op == 'stop':
            self.manual.stop('explicit_stop', wall)
            self.manual_prepare = None
            self.publish(dict(v=1, request=entry['request'], op='stop'))
            self.call(entry, 'stop')
        elif op == 'new_race':
            view = entry['view']
            self.publish(dict(v=1, request=entry['request'], op='new_race', session=view['session'],
                              race_epoch=view['race_epoch']))
            self.call(entry, 'stop')
        elif op in ('recovery_reset', 'resume_start'):
            self.publish(self.bound(entry, op))
        elif op == 'manual_visual_start':
            self.publish({**self.bound(entry, op), 'signal_confirmed': True})
        elif op == 'set_speed_limit':
            self.publish({**self.bound(entry, op), 'speed_limit_mps': entry['value']})
        elif op == 'prepare_start':
            self.publish({**self.bound(entry, 'acknowledge'), 'hazards_cleared': True, 'operator_reviewed': True})
        elif op == 'restore_race':
            record = self.core.restore_offer(time.time())
            if record is None:
                return self.core.mark(entry, wall, stage='rejected_by_adapter', adapter_reason='no_restorable_record')
            self.publish({**self.bound(entry, 'restore_race'), 'race_id': record['race_id'],
                          'side': record['side'], 'same_race_confirmed': True})
        self.core.mark(entry, wall, stage='sent', sent_wall=wall)

    def manual_mcu_fresh(self, wall):
        return fresh_status(self.core.mcu, self.get_clock().now().nanoseconds*1e-9,
                            self.core.mcu_wall, wall)

    def manual_abort(self, entry, reason, wall):
        self.manual.stop(reason, wall)
        self.manual_prepare = None
        self.manual.mark(entry, 'failed', reason, state='stopping' if self.manual.process else 'idle')
        self.call(entry, 'stop')
        self.core.mark(entry, wall, manual_reason=reason)

    def execute_manual(self, entry, wall):
        op = entry['op']
        if op == 'stop':
            self.manual.stop('explicit_manual_stop', wall)
            self.manual_prepare = None
            self.call(entry, 'stop')
            self.manual.mark(entry, 'stop_requested', 'stop_ack_and_state_are_separate')
            return
        reason = graph_reason(self)
        if not self.core.client_fresh(entry['client'], wall):
            reason = 'operator_link_lost'
        if reason or not self.manual_mcu_fresh(wall):
            self.manual.mark(entry, 'rejected', reason or 'fresh_mcu_required')
            return
        mcu = self.core.mcu
        if op == 'set_speed_limit':
            if mcu.get('state') not in (*LATCHED_MCU, 'DISARMED'):
                return self.manual.mark(entry, 'rejected', 'mcu_stop_required')
            self.manual.apply_limit(entry)
        elif op == 'prepare':
            if mcu.get('estop_held') is not False or mcu.get('state') not in (*LATCHED_MCU, 'DISARMED', 'ARMED'):
                return self.manual.mark(entry, 'rejected', 'manual_prepare_state_invalid')
            self.manual.begin_prepare(entry)
            self.manual_prepare = dict(entry=entry, deadline=wall+10., future=None,
                expected=None, waiting_until=0.)
        elif op == 'run':
            now = self.get_clock().now().nanoseconds*1e-9
            if (not valid_mcu(mcu, now, self.core.mcu_wall, wall)
                or mcu_identity(mcu) != self.manual.prepared_identity):
                return self.manual_abort(entry, 'prepared_mcu_changed_or_stale', wall)
            try:
                self.manual.launch(entry, wall)
            except BaseException as error:
                return self.manual_abort(entry, 'jog_launch_failed:'+str(error), wall)
        self.core.mark(entry, wall, stage=entry['stage'])

    def tick_manual_prepare(self, wall):
        job = self.manual_prepare
        if job is None:
            return
        entry, mcu = job['entry'], self.core.mcu or {}
        reason = graph_reason(self)
        if not self.core.client_fresh(entry['client'], wall): reason = 'operator_link_lost'
        if wall >= job['deadline']: reason = 'manual_prepare_timeout'
        if not self.manual_mcu_fresh(wall): reason = 'fresh_mcu_required'
        if mcu.get('estop_held') is not False: reason = 'physical_stop_held'
        if reason:
            return self.manual_abort(entry, reason, wall)
        future = job['future']
        if future is not None:
            if not future.done(): return
            try:
                response = future.result()
                accepted = response is not None and response.success is True
            except BaseException:
                accepted = False
            if not accepted: return self.manual_abort(entry, 'manual_prepare_service_rejected', wall)
            job['future'] = None
            job['waiting_until'] = wall+self.result_timeout_s
        state = mcu.get('state')
        if job['expected'] and state != job['expected']:
            if wall >= job['waiting_until']:
                return self.manual_abort(entry, 'manual_prepare_state_unconfirmed', wall)
            return
        job['expected'] = None
        if state == 'ARMED':
            now = self.get_clock().now().nanoseconds*1e-9
            if not valid_mcu(mcu, now, self.core.mcu_wall, wall):
                return self.manual_abort(entry, 'armed_feedback_not_ready', wall)
            self.manual.prepared(entry, mcu_identity(mcu))
            self.manual_prepare = None
            return
        step = 'clear_stop' if state in LATCHED_MCU else 'arm' if state == 'DISARMED' else None
        if step is None or not self.call(entry, step):
            return self.manual_abort(entry, 'manual_prepare_service_unavailable', wall)
        job['future'] = self.calls[-1][2]
        job['expected'] = 'DISARMED' if step == 'clear_stop' else 'ARMED'

    def tick_manual(self, wall):
        if self.manual.mode != 'manual':
            return  # 자율 구성에는 수동 publisher/자식/graph 폴링을 추가하지 않는다.
        while (entry := self.manual.next_action()) is not None:
            self.execute_manual(entry, wall)
        self.tick_manual_prepare(wall)
        if self.manual.process is not None:
            # 실행 중 graph 소유권은 실제 발행자인 기존 jog가 매 송신 전에 검사한다.
            # parent DDS discovery에서 새 child endpoint를 중복 판정하지 않는다.
            ended = self.manual.tick(wall, self.core.client_fresh(self.manual.client, wall))
            if ended is not None:
                self.call(ended, 'stop')  # 자식 정상 종료/실패와 무관하게 기존 STOP 서비스도 요청한다.
                self.core.mark(ended, wall, stage=ended['stage'])
        elif self.manual.prepared_identity is not None:
            now = self.get_clock().now().nanoseconds*1e-9
            reason = graph_reason(self)
            if (reason or not self.core.client_fresh(self.manual.client, wall)
                or not valid_mcu(self.core.mcu, now, self.core.mcu_wall, wall,
                                 identity=self.manual.prepared_identity)):
                entry = dict(op='manual_watchdog', request='manual-watchdog', mcu_steps=[])
                self.manual_abort(entry, reason or 'prepared_connection_or_mcu_lost', wall)

    def call(self, entry, name):
        client = self.mcu_clients[name]
        if not client.service_is_ready():
            self.core.mark(entry, mcu=name+':service_unavailable')
            return False
        request = Trigger.Request()
        self.calls.append((entry, name, client.call_async(request), time.monotonic()+MCU_CALL_TIMEOUT_S))
        self.core.mark(entry, mcu=name+':요청')
        return True

    def poll_calls(self, wall):
        for item in list(self.calls):
            entry, name, future, deadline = item
            if future.done():
                self.calls.remove(item)
                try:
                    result = future.result()
                except BaseException as error:
                    self.core.mark(entry, mcu=name+':응답 오류(결과 미확인)')
                    entry['mcu_steps'].append(dict(step=name, success=False, message=type(error).__name__, wall_s=wall))
                    if self.rearm is not None and self.rearm.get('future') is future:
                        self.rearm = None
                    continue
                if result is None:
                    self.core.mark(entry, mcu=name+':빈 응답(결과 미확인)')
                    entry['mcu_steps'].append(dict(step=name, success=False, message='empty_response', wall_s=wall))
                    if self.rearm is not None and self.rearm.get('future') is future:
                        self.rearm = None
                    continue
                self.core.mark(entry, mcu=f"{name}:{'확인' if result.success else '거절'} {result.message}")
                entry['mcu_steps'].append(dict(step=name, success=result.success, message=result.message, wall_s=wall))
                if self.rearm is not None and self.rearm.get('future') is future:
                    self.rearm['future'] = None
                    # 상태 토픽은 ARM 응답보다 늦다. 늦은 상태로 방금 올린 ARM을 다시 내리지 않는다.
                    self.rearm['armed'] = self.rearm.get('armed') or (name == 'arm' and result.success)
            elif wall >= deadline:
                self.calls.remove(item)
                self.mcu_clients[name].remove_pending_request(future)
                self.core.mark(entry, mcu=name+':응답 없음(결과 미확인)')
                if self.rearm is not None and self.rearm.get('future') is future:
                    self.rearm['future'] = None

    def rearm_step(self, job, wall):
        """MCU 상태로 다음 CLEAR/DISARM/ARM 하나만 부른다. 거절되면 짧게 재시도하며 큐에 쌓지 않는다."""
        mcu, race = self.core.mcu or {}, self.race()
        if job['future'] is not None or job.get('armed') or wall < job['next'] or not mcu:
            return
        state = mcu.get('state')
        name = ('clear_stop' if state in LATCHED_MCU else 'arm' if state == 'DISARMED' else
                'disarm' if state in ARMED_MCU and race.get('rearm_required') else None)
        if name is None:
            return
        before = len(self.calls)
        if self.call(job['entry'], name) and len(self.calls) > before:
            job['future'] = self.calls[-1][2]
        job['next'] = wall+MCU_RETRY_S

    def tick_start(self, wall):
        job = self.start
        if job is None:
            return
        race, entry = self.race(), job['entry']
        resume = race.get('resume')
        if race.get('session') != job['session']:
            self.core.mark(entry, wall, stage='cancelled', outcome='session_changed')
            self.start = self.rearm = None
            return
        if resume is None or resume.get('request') != entry['request']:
            last = race.get('last_resume') or {}
            if last.get('request') == entry['request']:
                self.core.mark(entry, wall, stage=last['outcome'], outcome=last.get('reason'))
                self.start = self.rearm = None
                return
            if wall >= job['status_deadline']:
                self.core.mark(entry, wall, stage='unknown_after_send', outcome='start_status_unconfirmed')
                self.start = self.rearm = None
                return
            if resume is not None:
                return
        if any(race.get(k) != entry['view'][k] for k in ('race_epoch', 'intervention')):
            return
        if not self.core.client_fresh(job['client'], wall):
            # 재연결 뒤 자동 재실행하지 않는다. 다시 확인한 상태의 새 요청만 받는다.
            self.publish(dict(v=1, request=entry['request'][:48]+'-cancel', op='cancel_start',
                              session=race['session'], race_epoch=race['race_epoch'],
                              start_request=entry['request'], reason='operator_link_lost'))
            self.core.mark(entry, wall, stage='cancelled', outcome='operator_link_lost')
            self.start = self.rearm = None
            return
        if wall-self.last_link >= LINK_PERIOD_S:
            self.last_link = wall
            # START 수락 응답은 주기 상태보다 먼저 도착할 수 있다. 이는 연결 확인뿐이며 ARM 허가가 아니다.
            self.link_publisher.publish(String(data=entry['request']))
        if resume is not None and resume.get('phase') == 'mcu':
            if self.rearm is None or self.rearm['entry'] is not entry:
                self.rearm = dict(entry=entry, next=0., future=None)
            self.rearm_step(self.rearm, wall)

    def tick_prepare(self, wall):
        job = self.rearm
        if job is None or job['entry']['op'] != 'prepare_start':
            return
        race = self.race()
        if wall >= job['deadline']:
            self.core.mark(job['entry'], wall, stage='unknown_after_send', outcome='mcu_rearm_timeout')
            self.rearm = None
        elif race.get('latch') is None and race.get('rearm_required') is False:
            self.core.mark(job['entry'], wall, stage='policy_accepted', outcome='armed_waiting_signal')
            self.rearm = None
        else:
            self.rearm_step(job, wall)

    def tick_record(self, wall):
        now = time.time()
        payload = self.core.record_payload(now)
        if not self.record_path or payload is None:
            return
        key = tuple(payload.get(k) for k in ('race_id', 'side', 'side_marker', 'session'))
        if key != self.last_written or wall-self.last_record >= RECORD_PERIOD_S:
            self.last_written, self.last_record = key, wall
            self.core.set_record(payload)
            self.record_writer.submit(json.dumps(payload, ensure_ascii=False))

    def tick_log(self):
        for row in self.core.drain_rows():
            self.log.submit(json.dumps(row, ensure_ascii=False, allow_nan=False, default=str))
        self.core.set_log_state(dict(dropped=self.log.dropped, failed=self.log.failed, bytes=self.log_bytes))

    def tick(self):
        wall = time.monotonic()
        if self.preview.needs_sync():self.preview.sync(self)
        while (entry := self.core.next_action()) is not None:
            self.execute(entry)
        self.poll_calls(wall)
        self.tick_manual(wall)
        self.tick_start(wall)
        self.tick_prepare(wall)
        self.core.expire(wall, self.result_timeout_s)
        self.tick_record(wall)
        self.tick_log()

    def destroy_node(self):
        self.preview.close(self)
        manual_active = (self.manual.mode == 'manual' and (self.manual.prepared_identity is not None
                         or self.manual_prepare is not None or self.manual.process is not None))
        if manual_active:
            self.call(dict(op='manual_shutdown', request='manual-shutdown', mcu_steps=[]), 'stop')
        self.manual.stop('panel_shutdown', time.monotonic())
        if self.manual.process is not None:
            deadline = time.monotonic()+4.
            while self.manual.process is not None and time.monotonic() < deadline:
                self.manual.tick(time.monotonic(), False)
                time.sleep(.02)
        self.server.shutdown()
        self.server.server_close()
        self.log.close()
        self.record_writer.close()
        if self.log_stream is not None:
            self.log_stream.close()
        return super().destroy_node()


def main(args=None):
    run_node(RaceOps, args=args)
