"""빠른 앞차 실패의 같은 저장 입력에서 기존 정지 재획득 OFF/ON을 비교한다.

사용: replay.py base|candidate
- base: 현 소스에서 기록 설정 그대로. 전체 출력 재현 확인.
- candidate: 현 소스에서 standing_wall_reacquire만 명시 선택.
다른 헤더 설정·execution_config·reset 순서를 바꾸지 않는다.
local_path 호출은 출력 변경 없이 감싸 43.4초 이후의 표현(wall_model)·진단만 기록한다.
저장 입력 대조이며 실제 재출발 증명이 아니다. Gazebo 미사용.
"""
import hashlib, json, sys, tempfile, zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DAY = HERE.parent
SOURCE = DAY/'side_corner_live_v1'/'right_parallel_first_curve'/'run1'/'follow_decisions.jsonl'
END_S = 40.0
TRACE_FROM_S = 32.0
LABEL = sys.argv[1]
assert LABEL == 'bridge'

TREE = ROOT
from dataclasses import replace

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
            'path_source_s', 'boundary_from_memory')
    return {'speed': out[0], 'steer': out[1], 'reason': det.get('reason'), 'behavior': det.get('behavior'),
            'mode_before': det.get('mode_before'), 'handoff_state': det.get('handoff_state'),
            'normal': {k: n.get(k) for k in keys if k in n}}


def main():
    raw_before = sha(SOURCE)
    controller = mc = cfg = None
    expected_seq = calls = resets = mismatches = 0
    rows = []
    with open(SOURCE, encoding='utf-8') as f:
        for line in f:
            e = json.loads(line)
            assert e['seq'] == expected_seq, (e['seq'], expected_seq)
            expected_seq += 1
            if e['kind'] == 'header':
                h = e['data']
                mc = MotionConfig(**h['motion_config'])
                cfg = FollowConfig(**h['follow_config'])
                assert cfg.standing_wall_reacquire
                assert not cfg.boundary_bridge
                cfg=replace(cfg,boundary_bridge=True)
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
            out = freeze(replay_call(controller, d['input'], mc))
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
        'follow_config': {k: getattr(cfg, k) for k in ('piecewise_wall_trial', 'standing_wall_reacquire', 'boundary_bridge',
                                                       'boundary_anchor_error_m', 'boundary_agreement_m',
                                                       'boundary_min_ahead_m', 'wall_offset_m')},
        'prefix_end_s': END_S, 'calls_replayed': calls, 'resets_replayed': resets,
        'mismatches_vs_recorded': mismatches, 'loaded_source_sha256': loaded,
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
