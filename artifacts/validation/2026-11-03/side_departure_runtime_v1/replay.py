"""격리된 실제 연결 코드의 OFF 보존과 ON 영향. 실제 궤적 재생이 아닌 판단 비교."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT/'build/side_departure_runtime_v1'
sys.path.insert(0, str(BASE))
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig

TRACES = dict(
    right=ROOT/'artifacts/validation/2026-11-03/right_reapproach_v1/run1/follow_decisions.jsonl',
    solo=ROOT/'artifacts/validation/2026-11-03/solo_post_free_v1/run1/follow_decisions.jsonl',
    irregular=ROOT/'artifacts/validation/2026-11-03/irregular_actor_v1/run1/follow_decisions.jsonl',
    merge=ROOT/'artifacts/validation/2026-11-02/official_traffic_v1/r1/follow_decisions.jsonl')


def diff(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        return [r for k in sorted(a.keys() | b.keys()) for r in diff(a.get(k), b.get(k), f'{path}.{k}')]
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)) and len(a) == len(b):
        return [r for i, (x, y) in enumerate(zip(a, b)) for r in diff(x, y, f'{path}[{i}]')]
    return [] if equal_output(a, b) else [dict(path=path, replay=a, recorded=b)]


def known_historical(rows):
    return all(d['path'].startswith(('[2].entry_space.candidates[', '[2].entry_admission.candidates['))
               and d['path'].endswith('].entry_path_conflict') and d['recorded'] is None
               and isinstance(d['replay'], dict) and d['replay'].get('rejected') is False
               and d['replay'].get('reason') == 'no_deep_current_path_conflict' for d in rows)


def main():
    summary = {}
    for label, path in TRACES.items():
        rows, history, footer, count = [], [], None, 0
        for line in path.open(encoding='utf-8'):
            event = json.loads(line); data = event['data']
            if event['seq'] != count or footer is not None:
                raise ValueError('trace order/footer')
            count += 1
            if event['kind'] == 'header':
                cfg, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
                enabled = cfg.pass_release_trial and not cfg.solo_route_only
                changed = replace(cfg, side_departure_trial=True) if enabled else cfg
                if 'execution_config' in data:
                    ex = HandoffExecutionConfig.from_mapping(data['execution_config'])
                    off, on = [BypassHandoffFollow(c, ex, allow_simulation_trial=True) for c in (cfg, changed)]
                else:
                    off, on = LocalFollow(cfg), LocalFollow(changed)
            elif event['kind'] == 'reset':
                off.reset(); on.reset()
            elif event['kind'] == 'footer':
                footer = data
            elif event['kind'] == 'call':
                raw = data['input']
                a = freeze(replay_call(off, raw, motion))
                differences = diff(comparable_decision(a), comparable_decision(data['output']))
                if differences:
                    if label != 'merge' or not known_historical(differences):
                        (HERE/f'{label}_replay_failure.json').write_text(json.dumps(dict(seq=event['seq'], differences=differences), indent=2), encoding='utf-8')
                        raise ValueError(f'OFF mismatch {label} seq={event["seq"]}')
                    history.append(dict(seq=event['seq'], differences=differences))
                b = freeze(replay_call(on, raw, motion))
                normal = b[2].get('normal') or b[2]
                dep = (normal.get('parallel_hold_trial') or {}).get('side_departure')
                rows.append(dict(seq=event['seq'], t=raw['reference_s'], dt=raw['dt_s'],
                                 original=a[:2], candidate=b[:2],
                                 original_state=a[2].get('handoff_state'), candidate_state=b[2].get('handoff_state'),
                                 departure=dep))
        if (footer is None or any(footer.get(k) != 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
                or footer.get('worker_alive') is not False
                or footer.get('written') != count-1 or footer.get('accepted') != count-1):
            raise ValueError('incomplete trace')
        changed_rows = [r for r in rows if not equal_output(r['original'], r['candidate'])
                        or r['original_state'] != r['candidate_state']]
        summary[label] = dict(calls=len(rows), candidate_enabled=enabled,
            changed_controls=len(changed_rows), first_changes=changed_rows[:2],
            new_zero=sum(r['candidate'][0] <= 1e-9 < r['original'][0] for r in rows),
            released_zero=sum(r['original'][0] <= 1e-9 < r['candidate'][0] for r in rows),
            zero_sample_s={k: sum(r['dt'] for r in rows if r[k][0] <= 1e-9) for k in ('original', 'candidate')},
            historical_diagnostic_additions=history, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        (HERE/f'{label}_replay.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
        print(json.dumps({k:v for k,v in summary[label].items() if k not in ('first_changes', 'historical_diagnostic_additions')} | dict(label=label)), flush=True)
    imported = [Path(m.__file__).resolve() for n,m in sys.modules.items()
                if n.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(m, '__file__', None)]
    if not imported or not all(p.is_relative_to(BASE.resolve()) for p in imported):
        raise ValueError('mixed runtime imports')
    (HERE/'replay.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
