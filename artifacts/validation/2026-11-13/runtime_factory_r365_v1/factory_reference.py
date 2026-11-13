"""명시 r353 정책을 r362 evaluator 계약에 연결하는 시험 전용 초안.

공통 src의 의존은 추가하지 않는다. 정책 원문은 보존하며 이 모듈만 시험
폴더를 로드한다. 미검증 실물 시각/엔코더를 승인하는 운영 factory가 아니다.
"""
import importlib.util
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
OLD = A.parent.parent/'2026-11-12'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def policy_support(output):
    # r334 RouteProcess의 원 지원 판정과 같은 모듈이다. 부모에서 계획은 설치하지 않는다.
    path = str(OLD/'retained_route_execution_r266_v1')
    if path not in sys.path:
        sys.path.insert(0, path)
    import candidate_policy
    return candidate_policy.supported(output)


class ReferenceEvaluator:
    def __init__(self, stage, header):
        if header.get('r365_policy') != 'r353' or stage not in ('plan', 'current'):
            raise ValueError('explicit r353 plan/current policy required')
        # r353와 같이 각 spawn 자식 안에만 해당 정책을 설치한다.
        self.stage = stage
        self.current_policy = None
        if stage == 'plan':
            boundary = load('r365_r278_boundary', OLD/'terminal_boundary_r278_v1/candidate.py')
            boundary.install()
        else:
            self.current_policy = load('r365_r352_current', A.parent/'solo_crawl_retry_r352_v1/candidate.py')
        from arena_autonomy.numeric_kernels import configure
        from arena_autonomy.decision_process import make_controller
        configure(header.get('numeric_execution', 'python'))
        self.follow, self.motion = make_controller(header)

    def reset(self, reason):
        from arena_autonomy.follow_trace import replay_reset
        replay_reset(self.follow, {'reason': reason})

    def __call__(self, data):
        from arena_autonomy.follow_trace import freeze, replay_call, restore_call
        if self.stage == 'plan':
            return freeze(replay_call(self.follow, data, self.motion))
        from arena_autonomy.lidar_motion import MotionUnavailable
        from arena_autonomy.scene_motion import motion_scene
        current = data['current']
        info, scan, history = restore_call(current, self.motion)
        try:
            # 원 overlap.evaluate_current와 동일: host_estimated 시간 허용을 추가하지 않는다.
            scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                                 epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
            result = self.current_policy.rebase(data['output'], data['submitted'], current, scene,
                history, self.follow.config, require_current_region=True)
        except (MotionUnavailable, ValueError) as error:
            result = dict(speed_mps=0., steering_rad=info['previous_steering_rad'],
                reference_s=info['reference_s'], reason='rebase_input_stop', input_error=str(error),
                independent_guard_required=True, runtime_authorized=False)
        if result is None:
            raise ValueError('unsupported current recheck')
        return freeze(result)


def evaluator_factory(stage, header):
    return ReferenceEvaluator(stage, header)
