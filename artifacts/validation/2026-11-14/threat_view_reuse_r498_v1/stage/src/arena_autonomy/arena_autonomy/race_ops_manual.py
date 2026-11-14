"""별도 수동 구성의 요청/단발 자식 관리. ROS publisher나 자동 ARM은 만들지 않는다."""
from collections import deque
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import uuid

from arena_autonomy.development_jog import MAX_STEERING_RAD, plan
from arena_autonomy.race_ops_core import token


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class ManualPanel:
    """HTTP는 요청만 넣고 ROS tick에서 실행한다. 재전송은 같은 receipt를 반환한다."""
    def __init__(self, mode='autonomous', maximum_mps=1.5, steering_limit_rad=.2,
                 output_root='build/panel_manual', spawn=subprocess.Popen):
        if mode not in ('autonomous', 'manual'):
            raise ValueError('operation_mode must be autonomous or manual')
        if not finite(maximum_mps) or maximum_mps < .05:
            raise ValueError('manual maximum speed must be finite and >= 0.05')
        if not finite(steering_limit_rad) or not 0 < steering_limit_rad <= MAX_STEERING_RAD:
            raise ValueError('manual steering limit exceeds supported vehicle command bound')
        self.mode, self.maximum_mps, self.limit_mps = mode, maximum_mps, maximum_mps
        self.steering_limit_rad = steering_limit_rad
        self.output_root, self.spawn = Path(output_root), spawn
        self.session, self.generation = uuid.uuid4().hex, 0
        self.lock = threading.RLock()
        self.entries, self.queue = {}, deque()
        self.state, self.reason = 'idle', 'explicit_manual_configuration_required' if mode != 'manual' else 'prepare_required'
        self.prepared_identity = None
        self.client = None
        self.process = self.job = self.stream = None
        self.last_heartbeat = 0.
        self.stop_since = None

    def binding(self):
        return dict(session=self.session, generation=self.generation)

    def snapshot(self):
        with self.lock:
            return dict(mode=self.mode, binding=self.binding(), state=self.state, reason=self.reason,
                maximum_mps=self.maximum_mps, limit_mps=self.limit_mps,
                pulse_limit_mps=min(.2, self.limit_mps), duration_max_s=2.,
                steering_limit_rad=self.steering_limit_rad,
                steering_limit_deg=math.degrees(self.steering_limit_rad),
                steering_reference='central_equivalent_wheel_angle_left_positive',
                prepared=self.prepared_identity is not None,
                commands=copy.deepcopy(list(self.entries.values())[-5:][::-1]))

    def submit(self, op, request, client, binding, value, wall):
        with self.lock:
            if request in self.entries:
                return copy.deepcopy(self.entries[request])
            reason = None
            if not token(request) or not token(client):
                reason = 'invalid_request_or_client'
            elif op not in ('prepare', 'run', 'stop', 'set_speed_limit'):
                reason = 'unknown_manual_operation'
            elif self.mode != 'manual':
                reason = 'explicit_manual_configuration_required'
            elif op != 'stop' and binding != self.binding():
                reason = 'stale_manual_view'
            elif op != 'stop' and (self.queue or self.state in ('preparing', 'running', 'stopping')):
                reason = 'manual_operation_busy'
            elif op == 'run' and self.prepared_identity is None:
                reason = 'prepare_required'
            elif op == 'set_speed_limit' and (not finite(value) or not .05 <= value <= self.maximum_mps):
                reason = 'speed_limit_out_of_range'
            elif op == 'run':
                try:
                    if type(value) is not dict or set(value) != {'speed_mps', 'seconds', 'steering_deg'}:
                        raise ValueError('manual request fields')
                    if not finite(value['steering_deg']):
                        raise ValueError('finite angle required')
                    plan('forward', value['speed_mps'], value['seconds'], min(.2, self.limit_mps),
                        steering_rad=math.radians(value['steering_deg']), max_steering_rad=self.steering_limit_rad)
                except (TypeError, ValueError):
                    reason = 'manual_value_out_of_range'
            entry = dict(op=op, request=request, client=client, binding=copy.deepcopy(binding), value=value,
                stage='rejected' if reason else 'queued', reason=reason, received_wall_s=wall, mcu_steps=[])
            self.entries[request] = entry
            if len(self.entries) > 32:
                del self.entries[next(iter(self.entries))]
            if reason is None:
                if op == 'stop':
                    self.generation += 1
                    self.prepared_identity = None
                    self.queue.appendleft(entry)
                else:
                    self.queue.append(entry)
            return copy.deepcopy(entry)

    def next_action(self):
        with self.lock:
            if not self.queue:
                return None
            entry = self.queue.popleft()
            if entry['op'] != 'stop' and entry['binding'] != self.binding():
                entry.update(stage='cancelled', reason='stale_manual_view')
                return None
            return entry

    def mark(self, entry, stage, reason, *, state=None):
        with self.lock:
            entry.update(stage=stage, reason=reason)
            self.reason = reason
            if state is not None:
                self.state = state

    def begin_prepare(self, entry):
        with self.lock:
            self.client = entry['client']
            self.prepared_identity = None
            self.mark(entry, 'preparing', 'explicit_clear_arm_pending', state='preparing')

    def prepared(self, entry, identity):
        with self.lock:
            self.prepared_identity = identity
            self.mark(entry, 'prepared', 'armed_zero_target_no_pulse', state='prepared')

    def apply_limit(self, entry):
        with self.lock:
            self.limit_mps = float(entry['value'])
            self.prepared_identity = None
            self.mark(entry, 'applied', 'speed_limit_set_held', state='idle')

    def launch(self, entry, wall):
        with self.lock:
            if self.process is not None or self.prepared_identity is None:
                raise RuntimeError('prepared exclusive jog required')
            value = entry['value']
            request = plan('forward', value['speed_mps'], value['seconds'], min(.2, self.limit_mps),
                steering_rad=math.radians(value['steering_deg']), max_steering_rad=self.steering_limit_rad)
            output = self.output_root / (self.session + '-' + entry['request'])
            output.parent.mkdir(parents=True, exist_ok=True)
            log_path = output.with_suffix('.log')
            self.stream = log_path.open('x', encoding='utf-8')
            node_name = 'panel_jog_' + uuid.uuid4().hex[:12]
            command = [sys.executable, '-m', 'arena_autonomy.development_jog', 'forward',
                '--speed', str(request['speed_mps']), '--seconds', str(request['duration_wall_s']),
                '--speed-limit', str(request['speed_limit_mps']), '--steering-deg', str(value['steering_deg']),
                '--max-steering-rad', str(self.steering_limit_rad), '--node-name', node_name,
                '--parent-heartbeat-stdin', '--send', '--output', str(output)]
            try:
                self.process = self.spawn(command, stdin=subprocess.PIPE, stdout=self.stream, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            except BaseException:
                self.stream.close(); self.stream = None
                raise
            self.job = dict(entry=entry, output=output, node_name=node_name,
                deadline=wall+request['duration_wall_s']+6., identity=self.prepared_identity)
            self.client = entry['client']
            self.prepared_identity = None
            self.stop_since = None
            self.last_heartbeat = wall-1.
            self.mark(entry, 'running', 'bounded_pulse_child_starting', state='running')
            entry['request_values'] = request

    def stop(self, reason, wall):
        with self.lock:
            self.generation += 1
            while self.queue:
                pending = self.queue.popleft()
                pending.update(stage='cancelled', reason=reason)
            self.prepared_identity = None
            self.reason = reason
            if self.process is None:
                self.state = 'idle'
                return
            if self.stop_since is None:
                self.stop_since = wall
                self.state = 'stopping'
                try:
                    self.process.stdin.write('STOP\n'); self.process.stdin.flush()
                    self.process.stdin.close()
                except (BrokenPipeError, OSError, ValueError):
                    pass

    def tick(self, wall, client_fresh):
        """종료 receipt를 돌려준다. 호출자는 별도 MCU STOP을 독립 요청한다."""
        with self.lock:
            process = self.process
            if process is None:
                return None
            if not client_fresh:
                self.stop('operator_link_lost', wall)
            if wall >= self.job['deadline']:
                self.stop('child_deadline', wall)
            if self.stop_since is None and wall-self.last_heartbeat >= .2:
                try:
                    process.stdin.write('HEARTBEAT\n'); process.stdin.flush()
                    self.last_heartbeat = wall
                except (BrokenPipeError, OSError, ValueError):
                    self.stop('child_pipe_lost', wall)
            if self.stop_since is not None and wall-self.stop_since > 3. and process.poll() is None:
                process.kill()
                self.reason = 'forced_child_cleanup_stop_unconfirmed'
            code = process.poll()
            if code is None:
                return None
            entry, output = self.job['entry'], self.job['output']
            try:
                result = json.loads((output/'result.json').read_text(encoding='utf-8'))
            except (OSError, ValueError):
                result = None
            complete = (code == 0 and isinstance(result, dict) and result.get('stop_ack') is True
                and result.get('fresh_estop_state_seen') is True and not result.get('error') and not result.get('errors'))
            self.mark(entry, 'completed' if complete else 'failed',
                'pulse_stopped_observed_not_physical_proof' if complete else self.reason if self.stop_since else 'child_failed', state='idle')
            entry['result'] = result
            entry['exit_code'] = code
            entry['output'] = str(output)
            self.stream.close(); self.stream = None
            try:
                if not process.stdin.closed: process.stdin.close()
            except (OSError, ValueError):
                pass
            self.process = self.job = None
            return entry


def graph_reason(node, expected_publisher=None):
    """자동 제어기는 0명령 상태여도 동시 publisher로 거절한다."""
    def peer(info): return info.node_namespace, info.node_name
    drive = node.get_publishers_info_by_topic('/drive')
    if any(info.node_name != expected_publisher or info.node_namespace != node.get_namespace() for info in drive):
        return 'autonomy_or_other_drive_publisher_present'
    if expected_publisher is None and drive:
        return 'drive_publisher_present'
    if expected_publisher is not None and len(drive) > 1:
        return 'duplicate_manual_drive_publisher'
    safe = node.get_publishers_info_by_topic('/drive/safe')
    readers = node.get_subscriptions_info_by_topic('/drive')
    hosts = node.get_subscriptions_info_by_topic('/drive/safe')
    status = node.get_publishers_info_by_topic('/actuation/status')
    if (len(safe) != 1 or safe[0].node_name != 'lidar_safety'
        or not any(peer(r) == peer(safe[0]) for r in readers)
        or not any(r.node_name in ('actuation_host', 'runtime_actuation_host') for r in hosts)
        or len(status) != 1):
        return 'independent_guard_or_host_missing'
    return None
