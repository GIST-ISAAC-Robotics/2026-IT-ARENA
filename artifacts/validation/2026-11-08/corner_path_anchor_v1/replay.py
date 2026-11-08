"""교통 한 랩 run1 저장 판단 입력 0~48초 prefix를 현재(앵커 오차 보완) 소스로 재생한다.

기준선은 traffic_reacquire_after_budget_v1/replay_result.json(수정 전 소스, 2,019판단 전체 일치)을 재사용한다.
헤더 config·reset 순서를 그대로 쓰고 옵션을 바꾸지 않는다. 저장 입력 대조이며 실제 재출발 증명이 아니다.
"""
import hashlib, json, sys, time, zipfile
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DAY = HERE.parent
SOURCE = DAY/'traffic_lap_budget_v1'/'two_actor_course_lap'/'run1'/'follow_decisions.jsonl'
BASELINE = DAY/'traffic_reacquire_after_budget_v1'/'replay_result.json'
CHANGED = ('src/arena_autonomy/arena_autonomy/local_path.py', 'src/arena_autonomy/arena_autonomy/local_follow.py')
END_S = 48.0
STALL_S = 43.82


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


def first_diff(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        if a.keys() != b.keys():
            return {'path': path, 'only_recorded': sorted(set(a)-set(b)), 'only_candidate': sorted(set(b)-set(a))}
        for k in a:
            d = first_diff(a[k], b[k], f'{path}/{k}')
            if d:
                return d
        return None
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return {'path': path, 'len_recorded': len(a), 'len_candidate': len(b)}
        for i, (x, y) in enumerate(zip(a, b)):
            d = first_diff(x, y, f'{path}/{i}')
            if d:
                return d
        return None
    return None if equal_output(a, b) else {'path': path, 'recorded': a, 'candidate': b}


def key(b):
    return f"{b['reason']}|{b['normal'].get('reason')}|{b['normal'].get('boundary_selection')}"


def main():
    baseline = json.loads(BASELINE.read_text(encoding='utf-8'))
    assert baseline['baseline_full_comparable_match'] and baseline['calls_replayed'] == 2019
    raw_sha_before = sha(SOURCE)
    assert raw_sha_before == baseline['source_sha256_before']
    controller = mc = cfg = None
    expected_seq = calls = resets = 0
    rows, changed = [], []
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
            out = freeze(replay_call(controller, d['input'], mc))
            calls += 1
            rec = freeze(d['output'])
            same = equal_output(comparable_decision(out), comparable_decision(rec))
            row = {'seq': e['seq'], 't': t, 'measured_speed_mps': d['input']['speed_mps'],
                   'same_as_recorded': same, 'recorded': brief(rec), 'candidate': brief(out)}
            rows.append(row)
            if not same:
                changed.append({**row, 'diff': first_diff(comparable_decision(rec), comparable_decision(out))})
    raw_sha_after = sha(SOURCE)
    calls_rows = [r for r in rows if 't' in r]

    def stats(name, rs):
        pos = [r for r in rs if r[name]['speed'] > 0]
        reasons = {}
        for r in rs:
            reasons[key(r[name])] = reasons.get(key(r[name]), 0)+1
        return {'calls': len(rs), 'positive_speed_calls': len(pos), 'first_positive_t': pos[0]['t'] if pos else None,
                'nonzero_steer_calls': sum(1 for r in rs if r[name]['steer'] != 0), 'reason_counts': reasons}
    transitions = {}
    for r in changed:
        k = f"{key(r['recorded'])} -> {key(r['candidate'])}"
        transitions[k] = transitions.get(k, 0)+1
    srcs = sorted({Path(m.__file__).resolve() for m in list(sys.modules.values())
                   if getattr(m, '__file__', None) and str(Path(m.__file__).resolve()).startswith(str(ROOT/'src'))})
    loaded = {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in srcs}
    with zipfile.ZipFile(HERE/'loaded_source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for p in srcs:
            z.write(p, str(p.relative_to(ROOT)).replace('\\', '/'))
    base_loaded = baseline['loaded_source_sha256']
    with zipfile.ZipFile(HERE/'before.zip') as z:
        before_changed = {p: hashlib.sha256(z.read(p)).hexdigest() for p in CHANGED}
    reuse = {
        'same_loaded_file_set': sorted(loaded) == sorted(base_loaded),
        'unchanged_files_equal': all(loaded[p] == base_loaded.get(p) for p in loaded if p not in CHANGED),
        'baseline_changed_files_equal_before_zip': all(base_loaded[p] == before_changed[p] for p in CHANGED),
        'changed_files_now': {p: loaded[p] for p in CHANGED},
        'changed_files_before': before_changed,
    }
    stall = [r for r in calls_rows if r['t'] >= STALL_S]
    result = {
        'source': str(SOURCE.relative_to(ROOT)).replace('\\', '/'),
        'source_sha256_before': raw_sha_before, 'source_sha256_after': raw_sha_after,
        'source_unchanged': raw_sha_before == raw_sha_after,
        'follow_config_piecewise_wall_trial': cfg.piecewise_wall_trial,
        'follow_config_boundary_anchor_error_m': cfg.boundary_anchor_error_m,
        'follow_config_wall_offset_m': cfg.wall_offset_m,
        'prefix_end_s': END_S, 'calls_replayed': calls, 'resets_replayed': resets,
        'baseline_reuse': reuse,
        'changed_calls_vs_recorded': len(changed),
        'first_change': changed[0] if changed else None,
        'change_transitions': transitions,
        'first_positive_change': next((r for r in changed if r['candidate']['speed'] > 0), None),
        'stall_window': {'from_s': STALL_S, 'to_s': END_S, 'recorded': stats('recorded', stall),
                         'candidate': stats('candidate', stall)},
        'loaded_source_sha256': loaded,
        'scope': 'stored decision inputs only; positive command is not proof of closed-loop restart; no Gazebo',
    }
    (HERE/'replay_result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    with open(HERE/'replay_rows.jsonl', 'w', encoding='utf-8', newline='\n') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False)+'\n')
    print(json.dumps({k: result[k] for k in ('calls_replayed', 'resets_replayed', 'changed_calls_vs_recorded',
                                              'source_unchanged', 'baseline_reuse')}, ensure_ascii=False))
    print(json.dumps(result['stall_window'], ensure_ascii=False))
    print(json.dumps(result['change_transitions'], ensure_ascii=False))
    print(json.dumps(result['first_change'], ensure_ascii=False)[:3000])
    ok = (result['source_unchanged'] and reuse['same_loaded_file_set'] and reuse['unchanged_files_equal']
          and reuse['baseline_changed_files_equal_before_zip'])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
