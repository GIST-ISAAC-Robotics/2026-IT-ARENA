"""피추월 저장 입력의 r24 실제 소스 연결 대조.
원 입력·설정·reset 순서를 보존하며 오프라인 후보와 비교한다. 실제 주행 증명이 아니다.
"""
import hashlib, json, sys, tempfile, zipfile, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DAY = HERE.parent
SOURCE = DAY/'rear_overtakes_newcourse_v1'/'left_rear_overtakes'/'run1'/'follow_decisions.jsonl'
END_S = 100.0
TRACE_FROM_S = 43.4
LABEL = 'connected'

TREE = ROOT

import arena_autonomy.local_follow as lf  # noqa: E402
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig  # noqa: E402
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402


state = {'t': None, 'calls': []}
original = lf.local_path


def traced(*args, **kwargs):
    path, info = original(*args, **kwargs)
    if state['t'] is not None and state['t'] >= TRACE_FROM_S:
        state['calls'].append({'side': args[1], 'reason': info.get('reason'), 'wall_model': info.get('wall_model'),
                               'has_path_anchor_error': 'path_anchor_error_m' in info,
                               'path_anchor_error_m': info.get('path_anchor_error_m'),
                               'wall_distance_m': info.get('wall_distance_m'),
                               'ahead_m': info.get('boundary_observed_ahead_m')})
    return path, info


lf.local_path = traced


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def brief(out):
    det = out[2] if isinstance(out[2], dict) else {}
    n = det.get('normal') if isinstance(det.get('normal'), dict) else {}
    keys = ('reason', 'behavior', 'boundary_candidate_reason', 'boundary_candidate_ahead_m', 'boundary_selection',
            'bilateral_reference', 'fresh_boundary_accepted', 'path_wall', 'preferred_wall', 'swept_clearance_m',
            'path_source_s', 'boundary_from_memory', 'selector_mode', 'selector_reason', 'parallel_hold_trial')
    return {'speed': out[0], 'steer': out[1], 'reason': det.get('reason'), 'behavior': det.get('behavior'),
            'mode_before': det.get('mode_before'), 'handoff_state': det.get('handoff_state'),
            'normal': {k: n.get(k) for k in keys if k in n}}


def main():
    raw_before = sha(SOURCE)
    controller = mc = cfg = None
    expected_seq = calls = resets = mismatches = 0
    rows = []
    costs=[]
    with open(SOURCE, encoding='utf-8') as f:
        for line in f:
            e = json.loads(line)
            assert e['seq'] == expected_seq, (e['seq'], expected_seq)
            expected_seq += 1
            if e['kind'] == 'header':
                h = e['data']
                mc = MotionConfig(**h['motion_config'])
                cfg = FollowConfig(**h['follow_config'])
                controller = BypassHandoffFollow(cfg, HandoffExecutionConfig.from_mapping(h['execution_config']),
                                                 allow_simulation_trial=True)
                continue
            if e['kind'] == 'reset':
                controller.reset()
                resets += 1
                rows.append({'seq': e['seq'], 'reset': e['data'].get('reason')})
                continue
            if e['kind'] != 'call':
                break
            d = e['data']
            t = d['input']['reference_s']
            if t > END_S:
                break
            state.update(t=t, calls=[])
            began=time.perf_counter()
            out = freeze(replay_call(controller, d['input'], mc))
            costs.append((time.perf_counter()-began)*1000)
            state['t'] = None
            calls += 1
            rec = freeze(d['output'])
            same = equal_output(comparable_decision(out), comparable_decision(rec))
            mismatches += not same
            row = {'seq': e['seq'], 't': t, 'measured_speed_mps': d['input']['speed_mps'], 'same_as_recorded': same,
                   'recorded': brief(rec), 'replayed': brief(out)}
            if state['calls']:
                row['local_path_calls'] = state['calls']
            if not same:
                row['comparable_replayed'] = comparable_decision(out)
                row['comparable_recorded'] = comparable_decision(rec)
            rows.append(row)
    srcs = sorted({Path(m.__file__).resolve() for m in list(sys.modules.values())
                   if getattr(m, '__file__', None) and str(Path(m.__file__).resolve()).startswith(str(TREE/'src'))})
    loaded = {str(p.relative_to(TREE)).replace('\\', '/'): sha(p) for p in srcs}
    result = {
        'label': LABEL, 'tree': str(TREE), 'arena_autonomy_file': lf.__file__,
        'source': str(SOURCE.relative_to(ROOT)).replace('\\', '/'),
        'source_sha256_before': raw_before, 'source_sha256_after': sha(SOURCE),
        'follow_config': {k: getattr(cfg, k) for k in ('piecewise_wall_trial', 'standing_wall_reacquire',
                                                       'boundary_anchor_error_m', 'boundary_agreement_m',
                                                       'boundary_min_ahead_m', 'wall_offset_m')},
        'prefix_end_s': END_S, 'calls_replayed': calls, 'resets_replayed': resets,
        'mismatches_vs_recorded': mismatches, 'scope': 'r24 connected source replay; compare with preserved artifact proposal; no closed-loop proof', 'loaded_source_sha256': loaded, 'decision_cost_ms': costs,
    }
    (HERE/f'replay_{LABEL}_result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    with open(HERE/f'replay_{LABEL}_rows.jsonl', 'w', encoding='utf-8', newline='\n') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False, default=lambda o: o.item() if hasattr(o, 'item') else str(o))+'\n')
    print(json.dumps({k: result[k] for k in ('label', 'arena_autonomy_file', 'follow_config', 'calls_replayed',
                                              'resets_replayed', 'mismatches_vs_recorded')}, ensure_ascii=False))
    return 0 if result['source_sha256_before'] == result['source_sha256_after'] else 1


if __name__ == '__main__':
    sys.exit(main())
