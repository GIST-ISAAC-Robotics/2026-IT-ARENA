"""기존 대표 입력의 원본 일치와 측방 후보의 영향 범위를 비교한다. 실제 궤적은 바뀌지 않는다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT/'build/side_departure_v1'
sys.path.insert(0, str(BASE))
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy import parallel_lane_trial as parallel

spec = importlib.util.spec_from_file_location('isolated_side_candidate', HERE/'candidate.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
TRACES = dict(
    right=ROOT/'artifacts/validation/2026-11-03/right_reapproach_v1/run1/follow_decisions.jsonl',
    selected=ROOT/'artifacts/validation/2026-11-03/solo_post_free_v1/run1/follow_decisions.jsonl',
    irregular=ROOT/'artifacts/validation/2026-11-03/irregular_actor_v1/run1/follow_decisions.jsonl',
    merge=ROOT/'artifacts/validation/2026-11-02/official_traffic_v1/r1/follow_decisions.jsonl',
)


def field_differences(a, b, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for key in sorted(a.keys() | b.keys()):
            out += field_differences(a.get(key), b.get(key), f"{path}.{key}")
        return out
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)) and len(a) == len(b):
        return [item for i, (x, y) in enumerate(zip(a, b)) for item in field_differences(x, y, f"{path}[{i}]")]
    return [] if equal_output(a, b) else [dict(path=path, replay=a, recorded=b)]


def main():
    old_limit, old_proposal = parallel.swept_limit, parallel.proposal

    def proposal(scene, baseline, speed, previous, dt, config):
        if 0 <= speed <= .35 and config.measurement_padding_m == .03:
            parallel.swept_limit = lambda *a, **k: candidate.candidate_limit(old_limit, *a, **k)
        try:
            return old_proposal(scene, baseline, speed, previous, dt, config)
        finally:
            parallel.swept_limit = old_limit

    # 경로 실수를 실행 중간에 발견하지 않도록 모두 확인한다.
    for path in TRACES.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    results = {}
    for label, path in TRACES.items():
        if label != 'merge':
            continue  # v2에서 완료한 동일 후보 284판단은 다시 실행하지 않는다.
        rows, seq, footer, timing_only, historical_additions = [], 0, None, [], []
        began = time.monotonic()
        for line in path.open(encoding='utf-8'):
            event = json.loads(line)
            if event['seq'] != seq or footer is not None:
                raise ValueError('입력 기록 순서 오류')
            seq += 1
            data = event['data']
            if event['kind'] == 'header':
                cfg, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
                if 'execution_config' in data:
                    execution = HandoffExecutionConfig.from_mapping(data['execution_config'])
                    baseline = BypassHandoffFollow(cfg, execution, allow_simulation_trial=True)
                    changed = BypassHandoffFollow(cfg, execution, allow_simulation_trial=True)
                else:
                    baseline, changed = LocalFollow(cfg), LocalFollow(cfg)
            elif event['kind'] == 'reset':
                baseline.reset(); changed.reset()
            elif event['kind'] == 'call':
                raw = data['input']
                before = freeze(replay_call(baseline, raw, motion))
                if not equal_output(before, data['output']):
                    diff = field_differences(before, data['output'])
                    if not equal_output(comparable_decision(before), comparable_decision(data['output'])):
                        normalized = field_differences(comparable_decision(before), comparable_decision(data['output']))
                        # 11/2 r1 뒤 추가된 경로-현재 반환 충돌 진단. 실제 거절/제어 변화는 허용하지 않는다.
                        known = all(d['path'].startswith(('[2].entry_space.candidates[', '[2].entry_admission.candidates['))
                                    and d['path'].endswith('].entry_path_conflict')
                                    and d['recorded'] is None and isinstance(d['replay'], dict)
                                    and d['replay'].get('rejected') is False
                                    and d['replay'].get('reason') == 'no_deep_current_path_conflict'
                                    for d in normalized)
                        if not known:
                            (HERE/f'{label}_replay_v5_failure.json').write_text(json.dumps(dict(seq=event['seq'], differences=diff), ensure_ascii=False, indent=2), encoding='utf-8')
                            raise ValueError(f'원본 재현 불일치 {label} seq={event["seq"]}')
                        historical_additions.append(dict(seq=event['seq'], differences=normalized))
                    else:
                        timing_only.append(dict(seq=event['seq'], differences=diff))
                parallel.proposal = proposal
                try:
                    after = freeze(replay_call(changed, raw, motion))
                finally:
                    parallel.proposal = old_proposal
                rows.append(dict(seq=event['seq'], t=raw['reference_s'], dt=raw['dt_s'], original=before[:2],
                                 candidate=after[:2], original_state=before[2].get('handoff_state'),
                                 candidate_state=after[2].get('handoff_state'),
                                 original_reason=(before[2].get('normal') or before[2]).get('reason'),
                                 candidate_reason=(after[2].get('normal') or after[2]).get('reason')))
            elif event['kind'] == 'footer':
                footer = data
        if (footer is None or any(footer.get(k) != 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
                or footer.get('worker_alive') is not False or footer.get('written') != seq-1 or footer.get('accepted') != seq-1):
            raise ValueError('완전하지 않은 입력')
        differences = [r for r in rows if not equal_output(r['original'], r['candidate'])
                       or r['original_state'] != r['candidate_state']]
        result = dict(calls=len(rows), timing_only_differences=timing_only, historical_diagnostic_additions=historical_additions, changed_controls=len(differences), first_changes=differences[:5],
                      new_zero=sum(r['candidate'][0] <= 1e-9 < r['original'][0] for r in rows),
                      released_zero=sum(r['original'][0] <= 1e-9 < r['candidate'][0] for r in rows),
                      zero_sample_s={key:sum(r['dt'] for r in rows if r[key][0] <= 1e-9) for key in ('original','candidate')},
                      source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), wall_s=time.monotonic()-began)
        (HERE/f'{label}_full_v5_decisions.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
        results[label] = result
        print(json.dumps(dict(label=label, **result)), flush=True)
    imports = [Path(m.__file__).resolve() for n, m in sys.modules.items()
               if n.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(m, '__file__', None)]
    if not all(p.is_relative_to(BASE.resolve()) for p in imports):
        raise ValueError('혼합 소스 import')
    (HERE/'full_replay_v5.json').write_text(json.dumps(dict(
        scope='원래 궤적의 저장 입력에 대한 대체 판단. 독립 보호/폐루프/실차 증명 아님.',
        results=results, right_previous_evidence='full_replay_v2.log / right_full_v2_decisions.json'), ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
