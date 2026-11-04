"""정지 상대 구간에서 최초 양수 속도 허가까지 비교. 원래 prefix는 보존한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
ISOLATED = ROOT/'build/side_departure_v1'
sys.path.insert(0, str(ISOLATED))
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy import parallel_lane_trial as parallel

spec = importlib.util.spec_from_file_location('isolated_side_candidate', HERE/'candidate.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
TRACE = ROOT/'artifacts/validation/2026-11-03/right_reapproach_v1/run1/follow_decisions.jsonl'


def main():
    original_limit, original_proposal = parallel.swept_limit, parallel.proposal

    def proposal(scene, baseline, speed, previous, dt, config):
        # 선택형 원본의 나머지 관측/직선/조향/속도/확인 조건은 그대로 호출한다.
        if 0 <= speed <= .35 and config.measurement_padding_m == .03:
            parallel.swept_limit = lambda *a, **k: candidate.candidate_limit(original_limit, *a, **k)
        try:
            return original_proposal(scene, baseline, speed, previous, dt, config)
        finally:
            parallel.swept_limit = original_limit

    count, result = 0, None
    green = json.loads((TRACE.parent/"report.json").read_bytes())["evaluation"]["t_green"]
    for line in TRACE.open(encoding='utf-8'):
        event = json.loads(line); data = event['data']
        if event['kind'] == 'header':
            cfg, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
            execution = HandoffExecutionConfig.from_mapping(data['execution_config'])
            baseline = BypassHandoffFollow(cfg, execution, allow_simulation_trial=True)
            changed = BypassHandoffFollow(cfg, execution, allow_simulation_trial=True)
        elif event['kind'] == 'reset':
            baseline.reset(); changed.reset()
        elif event['kind'] == 'call':
            raw = data['input']
            if raw['reference_s'] > green+1.5:
                break
            before = freeze(replay_call(baseline, raw, motion))
            if not equal_output(before, data['output']):
                raise ValueError(f'원본 재현 불일치 seq={event["seq"]}')
            parallel.proposal = proposal
            try:
                after = freeze(replay_call(changed, raw, motion))
            finally:
                parallel.proposal = original_proposal
            count += 1
            if before[0] <= 1e-9 and after[0] > 1e-9:
                result = dict(seq=event['seq'], reference_s=raw['reference_s'], original=before[:2],
                              candidate=after[:2], measured_speed_mps=raw['speed_mps'],
                              normal_before=before[2].get('normal'), normal_after=after[2].get('normal'))
                break
    imported = [Path(m.__file__).resolve() for n, m in sys.modules.items()
                if n.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(m, '__file__', None)]
    if not imported or not all(p.is_relative_to(ISOLATED.resolve()) for p in imported):
        raise ValueError('혼합 소스 import')
    output = dict(calls=count, first_control_difference=result, source_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
                  scope='상대 정지 구간의 최초 양수 속도 허가까지 격리 저장 입력 비교. 중간 조향 차이 뒤에도 원래 입력을 사용한 반사실이며 독립 보호층/폐루프/실차 및 채택 증명 아님.')
    (HERE/'forward_prefix.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(dict(calls=count, found=result is not None,
                         first={k: v for k, v in result.items() if not k.startswith('normal_')} if result else None)))


if __name__ == '__main__':
    main()
