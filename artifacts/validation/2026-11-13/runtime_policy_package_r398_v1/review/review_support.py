"""r398 제한 비교용. 운영 import/정책과 별개의 원 체인 로더 및 입력 준비."""
import copy
from dataclasses import asdict, is_dataclass
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
ROOT = BASE.parents[3]
OLD = BASE.parent.parent / '2026-11-12'
TRACE = BASE.parent / 'released_cad_drive_r377_v1/run1/follow_decisions.jsonl'
VEHICLE = ROOT / 'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
PACKAGE = BASE / 'staged/src/arena_autonomy/arena_autonomy/race_runtime_policy'
sys.path[:0] = [str(BASE / 'staged/src/arena_autonomy'),
               str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface')]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encoded(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def corpus():
    """기록된 완료 계획 중 중복 sequence를 제거한 prefix와 current 223개.

    실제 실행 중 결과가 기록되지 않은 계산을 복원하지 않는다. 원 expected는
    참고로 보존할 뿐 최종 기하에서 새로 계산한 baseline의 정답으로 쓰지 않는다.
    """
    from arena_autonomy.runtime_dual_process import plan_reference
    header = None
    plans, currents, seen = [], [], set()
    reason = None
    with TRACE.open(encoding='utf8') as stream:
        for line in stream:
            row = json.loads(line)
            data = row['data']
            if row['kind'] == 'header':
                header = data
            elif row['kind'] == 'reset':
                reason = data.get('reason')
            elif row['kind'] == 'call':
                computation = data['compute']
                identity = (computation['generation'], computation['sequence'])
                if identity not in seen:
                    seen.add(identity)
                    assert computation['input'] == data['input']
                    plans.append(dict(data=data['input'], generation=identity[0],
                        sequence=identity[1], reset_reason=reason, trace_seq=row['seq']))
                    reason = None
                current = data.get('current_rebase')
                if current:
                    plan = current['compute']['plan']
                    currents.append(dict(data=dict(output=plan['output'],
                        submitted=plan_reference(plan['input']), current=current['input']),
                        generation=computation['generation'], sequence=len(currents)+1,
                        reset_reason=None, trace_seq=row['seq'],
                        recorded_output=current['output']))
    assert header is not None and header['schema'] == 5 and len(currents) == 223
    return header, plans, currents


def selected_header(recorded, numeric='python', policy='verified_only'):
    import yaml
    from arena_autonomy.vehicle_geometry import planning_parameters, preview_parameters
    vehicle = yaml.safe_load(VEHICLE.read_bytes())['vehicle']
    planning, sender = planning_parameters(vehicle), preview_parameters(vehicle)
    header = copy.deepcopy(recorded)
    cfg = header['follow_config']
    # r377 당시 존재한 주요 기하와 일치해야 한다. 새 필드는 양쪽에 똑같이 명시.
    expected = dict(length_m=planning['vehicle_length_m'], width_m=planning['vehicle_width_m'],
        wheelbase_m=sender['wheelbase_m'], max_steering_rad=sender['max_steering_angle_rad'])
    assert all(cfg[key] == value for key, value in expected.items())
    cfg.update(track_m=planning['vehicle_track_width_m'],
        wheel_radius_m=planning['vehicle_wheel_radius_m'], wheel_width_m=planning['vehicle_wheel_width_m'])
    assert header['motion_config']['wheel_radius_m'] == sender['motion_wheel_radius_m']
    assert header['motion_config']['rear_axle_x_m'] == sender['motion_rear_axle_x_m']
    header.update(numeric_execution=numeric, runtime_policy='cad_runtime_r377_v1',
                  runtime_time_policy=policy)
    return header


def estimated_call(source):
    """합성 same-values 반례. 모든 숫자/이력/원천 시각은 그대로다."""
    out = copy.deepcopy(source)
    assert out['timing_verified'] is True
    assert out.get('timing_policy', 'verified_only') == 'verified_only'
    assert out['scan']['measurements']['time_basis'] == 'simulation_acquisition'
    out['timing_verified'] = False
    out['timing_policy'] = 'host_estimated_v1'
    out['scan']['measurements']['time_basis'] = 'unverified_estimate'
    restored = copy.deepcopy(out)
    restored['timing_verified'] = True
    if 'timing_policy' in source:
        restored['timing_policy'] = source['timing_policy']
    else:
        del restored['timing_policy']
    restored['scan']['measurements']['time_basis'] = 'simulation_acquisition'
    assert encoded(restored) == encoded(source)
    return out


def estimated_data(stage, data):
    if stage == 'plan':
        return estimated_call(data)
    answer = copy.deepcopy(data)
    answer['current'] = estimated_call(data['current'])
    return answer


def state_value(value):
    """r379에서 검증한 선택기 상태 정규화: set의 원소·종류도 보존."""
    from arena_autonomy.follow_trace import freeze
    if is_dataclass(value):
        return state_value(asdict(value))
    if isinstance(value, (set, frozenset)):
        return {'__container_type__': type(value).__name__,
                'items': sorted((state_value(v) for v in value), key=encoded)}
    if isinstance(value, dict):
        return {k: state_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [state_value(v) for v in value]
    if hasattr(value, '__dict__'):
        return state_value(vars(value))
    return freeze(value)


def checkpoint(follow):
    normal = getattr(follow, 'normal', follow)
    # 선택기 전체 상태 + 경로/정지/재개 상태. 전체 객체의 순환 참조는 가져오지 않는다.
    names = ('path', 'pending', 'path_ref', 'path_source', 'pending_ref', 'recent_path',
             'pending_count', 'release_count', 'last_source', 'last_ref', 'epoch', 'side',
             'path_wall', 'held', 'hold_reason', 'partial_epoch')
    return dict(selector=state_value(normal.pass_selector),
                path=state_value({name: getattr(normal, name) for name in names}),
                handoff=state_value({name: getattr(follow, name) for name in
                    ('mode', 'last_key', 'last_ref', 'reacquire')}))


class OriginalEvaluator:
    """r377 low_solo_worker와 같은 r278+r361 / r375 체인, 같은 현행 common.

    옛 worker의 파일 receipt/ROS 연결 대신 factory 계약만 감싼다. 원 정책
    모듈은 importlib로 직접 읽고 파일이나 _geometry를 고치지 않는다.
    """
    def __init__(self, stage, header):
        from arena_autonomy.numeric_kernels import configure
        from arena_autonomy.decision_process import make_controller
        self.stage = stage
        self.boundary = self.scope = None
        configure(header['numeric_execution'])
        if stage == 'plan':
            self.boundary = load('r398_original_terminal', OLD / 'terminal_boundary_r278_v1/candidate.py')
            self.boundary.install()
            released = load('r398_original_released', BASE.parent / 'released_crawl_r361_v1/candidate.py')
            self.scope = released.enabled()
            self.scope.__enter__()
        else:
            sys.path.insert(0, str(OLD / 'retained_route_execution_r266_v1'))
            self.policy = load('r398_original_current', BASE.parent / 'released_current_r375_v1/candidate.py')
        try:
            self.follow, self.motion = make_controller(header)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.scope is not None:
            self.scope.__exit__(None, None, None)
            self.scope = None
        if self.boundary is not None:
            self.boundary.uninstall()
            self.boundary = None

    def reset(self, reason):
        from arena_autonomy.follow_trace import replay_reset
        replay_reset(self.follow, {'reason': reason})

    def __call__(self, data):
        from arena_autonomy.follow_trace import freeze, replay_call, restore_call
        from arena_autonomy.scene_motion import motion_scene
        from arena_autonomy.lidar_motion import MotionUnavailable
        if self.stage == 'plan':
            return freeze(replay_call(self.follow, data, self.motion))
        current = data['current']
        info, scan, history = restore_call(current, self.motion)
        try:
            scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                                 epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
            result = self.policy.rebase(data['output'], data['submitted'], current, scene,
                                       history, self.follow.config, require_current_region=True)
        except (MotionUnavailable, ValueError) as error:
            result = dict(speed_mps=0., steering_rad=info['previous_steering_rad'],
                reference_s=info['reference_s'], reason='rebase_input_stop', input_error=str(error),
                independent_guard_required=True, runtime_authorized=False)
        if result is None:
            raise ValueError('unsupported current recheck')
        return freeze(result)


class AuditEvaluator:
    """오프라인 factory wrapper만 상태/입력 불변 진단을 더한다."""
    def __init__(self, stage, header):
        self.stage = stage
        if header['review_chain'] == 'original':
            self.inner = OriginalEvaluator(stage, header)
        else:
            from arena_autonomy.race_runtime_policy.evaluator import evaluator_factory
            self.inner = evaluator_factory(stage, header)

    def reset(self, reason):
        self.inner.reset(reason)

    def close(self):
        self.inner.close()

    def __call__(self, data):
        before = encoded(data)
        output = self.inner(data)
        assert before == encoded(data), 'evaluator mutated its owned input'
        return dict(output=output, state=checkpoint(self.inner.follow))


def audit_factory(stage, header):
    return AuditEvaluator(stage, header)
