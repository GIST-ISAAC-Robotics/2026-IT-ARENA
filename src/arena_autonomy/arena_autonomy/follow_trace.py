"""선택형 G1 판단 단계 기록. ROS/정답 입력 없이 같은 순서의 판단을 재생한다.

스캔과 이미 변환된 MotionHistory를 판단 시점에 복사한다. 원시 센서 콜백이나
RGB 허가·출력 제한·독립 보호층 전체의 재생은 아니다. 기본 실행에는 연결하지 않는다.
"""
from dataclasses import asdict
import json
import math
from numbers import Real
from pathlib import Path
from types import SimpleNamespace

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_vehicle_interface.queued_log import QueuedLog


def freeze(value):
    """스레드 밖에서 공유 객체를 복사하고 NaN/+Inf/-Inf를 구별해 보존한다."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Real):
        if not math.isfinite(value):
            return {'nonfinite': 'nan' if math.isnan(value) else ('inf' if value > 0 else '-inf')}
        return int(value) if isinstance(value, int) else float(value)
    if isinstance(value, dict):
        return {str(k): freeze(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [freeze(v) for v in value]
    if hasattr(value, 'tolist'):
        return freeze(value.tolist())
    raise TypeError(f'unsupported trace value: {type(value).__name__}')


def thaw(value):
    if isinstance(value, dict):
        if set(value) == {'nonfinite'}:
            if value['nonfinite'] not in ('nan', 'inf', '-inf'):
                raise ValueError('invalid nonfinite encoding')
            return float(value['nonfinite'])
        return {k: thaw(v) for k, v in value.items()}
    return [thaw(v) for v in value] if isinstance(value, list) else value


def snapshot_call(scan, history, reference_s, epoch, speed, side, steering, dt,
                  verified, rear_half_angle):
    return freeze({
        'scan': {'sec': scan.header.stamp.sec, 'nanosec': scan.header.stamp.nanosec,
                 'frame_id': scan.header.frame_id, 'ranges': list(scan.ranges),
                 **{k: getattr(scan, k) for k in ('angle_min', 'angle_increment', 'range_min',
                                                 'range_max', 'time_increment')}},
        # values는 반지름/기어비/바이어스 적용 뒤 값. 재생에서 중복 환산하지 않는다.
        'history': {k: {'times': list(getattr(history, k).times),
                        'values': list(getattr(history, k).values)} for k in ('wheels', 'gyro')},
        'reference_s': reference_s, 'epoch': epoch, 'speed_mps': speed, 'side': side,
        'previous_steering_rad': steering, 'dt_s': dt, 'timing_verified': verified,
        'rear_half_angle_deg': rear_half_angle})


def controller_mode(follow):
    from arena_autonomy.bypass_handoff import BypassHandoffFollow
    if isinstance(follow, BypassHandoffFollow):
        return 'g1_g2_handoff_trial'
    from arena_autonomy.r0_plus import R0Plus
    from arena_autonomy.bypass_entry_follow import BypassEntryFollow
    if isinstance(follow, BypassEntryFollow):
        return 'g2_entry_trial'
    from arena_autonomy.m3_execution_trial import M3ExecutionFollow
    if isinstance(follow, M3ExecutionFollow):
        return 'm3_execution_trial'
    return 'r0_plus_candidate' if isinstance(follow, R0Plus) else 'g1_follow'


class FollowTrace:
    def __init__(self, path, motion_config, follow_config, capacity=128, *, mode='g1_follow', execution_config=None):
        if mode not in ('g1_follow', 'r0_plus_candidate', 'm3_execution_trial', 'g2_entry_trial', 'g1_g2_handoff_trial'):
            raise ValueError('unsupported decision trace mode')
        if mode == 'm3_execution_trial':
            from arena_autonomy.m3_execution_trial import ExecutionConfig
            if not isinstance(execution_config, ExecutionConfig):
                raise ValueError('M3 trace requires typed execution config')
        elif mode == 'g2_entry_trial':
            from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
            if not isinstance(execution_config, BypassExecutionConfig):
                raise ValueError('G2 trace requires typed execution config')
        elif mode == 'g1_g2_handoff_trial':
            from arena_autonomy.bypass_handoff import HandoffExecutionConfig
            if not isinstance(execution_config, HandoffExecutionConfig):
                raise ValueError('handoff trace requires typed execution config')
        elif execution_config is not None:
            raise ValueError('execution config requires M3 trace')
        target = Path(path)
        if not target.is_absolute():
            raise ValueError('g1_trace_path must be absolute')
        # 기존 증거 덮어쓰기 금지. 디렉터리는 호출자가 명시적으로 준비한다.
        self.stream = target.open('x', encoding='utf-8')
        self.sequence = 0
        self.capture_errors = 0
        self.closed = False
        self.writer = QueuedLog(self._write, capacity=capacity)
        identity = ({'schema': 1, 'scope': 'g1_decision_stage_only'} if mode == 'g1_follow' else
                    {'schema': 2, 'scope': 'r0plus_decision_stage_only', 'controller_mode': mode})
        if mode == 'm3_execution_trial':
            identity = {'schema': 3, 'scope': 'm3_execution_decision_stage_only', 'controller_mode': mode,
                        'execution_config': asdict(execution_config)}
        if mode == 'g2_entry_trial':
            identity = {'schema': 4, 'scope': 'g2_entry_decision_stage_only', 'controller_mode': mode,
                        'execution_config': asdict(execution_config)}
        if mode == 'g1_g2_handoff_trial':
            identity = {'schema': 5, 'scope': 'g1_g2_handoff_decision_stage_only', 'controller_mode': mode,
                        'execution_config': asdict(execution_config)}
        self.emit('header', {**identity,
                            'history_units': {'wheels': 'm/s_after_conversion', 'gyro': 'rad/s_after_bias'},
                            'motion_config': asdict(motion_config), 'follow_config': asdict(follow_config)})

    def _write(self, event):
        self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n')

    def emit(self, kind, data):
        if self.closed:
            self.capture_errors += 1
            return
        seq = self.sequence
        self.sequence += 1
        try:
            self.writer.submit({'seq': seq, 'kind': kind, 'data': freeze(data)})
        except Exception:
            # 기록 실패로 제어 출력을 바꾸지 않는다. footer에서 전체 기록을 무효화한다.
            self.capture_errors += 1

    def record_call(self, scan, history, reference_s, epoch, speed, side, steering, dt,
                    verified, rear_half_angle, output):
        try:
            inputs = snapshot_call(scan, history, reference_s, epoch, speed, side, steering,
                                   dt, verified, rear_half_angle)
            self.emit('call', {'input': inputs, 'output': output})
        except Exception:
            self.capture_errors += 1

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.writer.close():
            status = self.writer.snapshot()
            try:
                self._write({'seq': self.sequence, 'kind': 'footer',
                             'data': {**status, 'capture_errors': self.capture_errors}})
            finally:
                self.stream.close()
        # 늦은 writer와 같은 파일을 동시에 닫지 않는다. footer 없는 기록은 재생 불합격.


def restore_call(data, motion_config):
    data = thaw(data)
    raw = dict(data['scan'])
    header = SimpleNamespace(stamp=SimpleNamespace(sec=raw.pop('sec'), nanosec=raw.pop('nanosec')),
                             frame_id=raw.pop('frame_id'))
    scan = SimpleNamespace(header=header, **raw)
    history = MotionHistory(motion_config)
    for name in ('wheels', 'gyro'):
        series = data['history'][name]
        times, values = series['times'], series['values']
        if (len(times) != len(values) or len(times) > 10_000
                or any(not math.isfinite(v) for v in times+values)
                or any(t < 0 for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ValueError('invalid captured history')
        # 이미 환산된 값이다. add_mean_wheel/add_gyro의 환산을 중복 적용하지 않는다.
        target = getattr(history, name)
        target.times, target.values = list(times), list(values)
    return data, scan, history


def replay_call(follow, data, motion_config):
    data, scan, history = restore_call(data, motion_config)
    try:
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        return follow.command(scene, history, data['speed_mps'], data['side'],
                              data['previous_steering_rad'], data['dt_s'],
                              **({'raw_scan':scan} if follow.config.partial_observation_policy!='strict' else {}))
    except (MotionUnavailable, ValueError) as error:
        follow.reset()
        return 0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD',
                       'input_error': str(error), 'mode': controller_mode(follow)}


def equal_output(a, b, tolerance=1e-10):
    # 비교 보고서의 허용 오차일 뿐 주행 판정선은 아니다.
    if isinstance(a, Real) and not isinstance(a, bool) and isinstance(b, Real) and not isinstance(b, bool):
        return math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance)
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(equal_output(a[k], b[k], tolerance) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(equal_output(x, y, tolerance) for x, y in zip(a, b))
    return type(a) is type(b) and a == b


def comparable_decision(output):
    """표면 정합의 벽시계 비용만 비교에서 제외한다. 원시 기록/판단값은 보존한다."""
    import copy
    result=copy.deepcopy(output)
    if isinstance(result,(list,tuple)) and len(result)==3 and isinstance(result[2],dict):
        fits=[]
        components=[result[2]]
        if result[2].get('mode') == 'g1_g2_handoff_trial' and isinstance(result[2].get('normal'),dict):
            components.append(result[2]['normal'])
        for component in components:
            observation=component.get('partial_observation')
            if isinstance(observation,dict):
                fits.append(observation.get('surface_correspondence'))
                anchor=observation.get('anchor_diagnostic')
                if isinstance(anchor,dict):
                    fits.append(anchor.get('fit'))
        # 이름이 같은 다른 수치를 재귀적으로 지우지 않는다. 이 두 위치의
        # 표면 정합 wall-clock 비용만 비결정적 진단값이며 제어 출력은 아니다.
        for fit in fits:
            if not isinstance(fit,dict) or 'processing_ms' not in fit:
                continue
            value=fit['processing_ms']
            if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value) or value<0:
                raise ValueError('invalid surface processing cost')
            fit['processing_ms']=0.
    return result


def replay_events(events):
    count, calls, mismatches, footer = 0, 0, [], None
    follow = motion = None
    scope = None
    outcomes = []
    for event in events:
        if footer is not None or event.get('seq') != count:
            raise ValueError('trace gap, reorder, or trailing data')
        count += 1
        kind, data = event['kind'], event['data']
        if count == 1:
            standard = data.get('schema') == 1 and data.get('scope') == 'g1_decision_stage_only'
            plus = (data.get('schema') == 2 and data.get('scope') == 'r0plus_decision_stage_only'
                    and data.get('controller_mode') == 'r0_plus_candidate')
            m3 = (data.get('schema') == 3 and data.get('scope') == 'm3_execution_decision_stage_only'
                  and data.get('controller_mode') == 'm3_execution_trial')
            g2 = (data.get('schema') == 4 and data.get('scope') == 'g2_entry_decision_stage_only'
                  and data.get('controller_mode') == 'g2_entry_trial')
            handoff = (data.get('schema') == 5 and data.get('scope') == 'g1_g2_handoff_decision_stage_only'
                       and data.get('controller_mode') == 'g1_g2_handoff_trial')
            if kind != 'header' or not (standard or plus or m3 or g2 or handoff):
                raise ValueError('unsupported trace header')
            scope = data['scope']
            motion = MotionConfig(**data['motion_config'])
            follow_type = LocalFollow
            if plus:
                from arena_autonomy.r0_plus import R0Plus
                follow_type = R0Plus
            if m3:
                from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
                follow = M3ExecutionFollow(FollowConfig(**data['follow_config']),
                    ExecutionConfig(**data['execution_config']), allow_simulation_trial=True)
            elif g2:
                from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
                follow = BypassEntryFollow(FollowConfig(**data['follow_config']),
                    BypassExecutionConfig(**data['execution_config']), allow_simulation_trial=True)
            elif handoff:
                from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
                follow = BypassHandoffFollow(FollowConfig(**data['follow_config']),
                    HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
            else:
                follow = follow_type(FollowConfig(**data['follow_config']))
        elif kind == 'reset':
            follow.reset()
        elif kind == 'call':
            actual = freeze(replay_call(follow, data['input'], motion))
            if not equal_output(comparable_decision(actual), comparable_decision(data['output'])):
                mismatches.append({'seq': event['seq'], 'expected': data['output'], 'actual': actual})
            calls += 1
            outcomes.append({'seq': event['seq'], 'reference_s': data['input']['reference_s'],
                             'speed_mps': actual[0], 'steering_rad': actual[1], 'details': actual[2]})
        elif kind == 'footer':
            footer = data
        else:
            raise ValueError('unknown trace event')
    if footer is None or calls == 0:
        raise ValueError('missing footer or no decision calls')
    if (footer.get('capture_errors') != 0 or footer.get('failed') != 0 or footer.get('dropped') != 0
            or footer.get('pending') != 0 or footer.get('worker_alive') is not False
            or footer.get('written') != count-1 or footer.get('accepted') != count-1):
        raise ValueError('incomplete trace capture')
    return {'scope': scope, 'calls': calls, 'matched': calls-len(mismatches),
            'mismatches': mismatches, 'passed': not mismatches, 'outcomes': outcomes}
