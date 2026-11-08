"""replay_base/candidate 결과를 대조해 summary.json을 만든다. 입력 파일만 읽는다."""
import hashlib, json, sys, zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
CHANGED = ('src/arena_autonomy/arena_autonomy/local_path.py', 'src/arena_autonomy/arena_autonomy/local_follow.py')
STALL_S, TRANSITION_S = 43.82, 44.02


def load(label):
    result = json.loads((HERE/f'replay_{label}_result.json').read_text(encoding='utf-8'))
    rows = [json.loads(l) for l in open(HERE/f'replay_{label}_rows.jsonl', encoding='utf-8')]
    return result, [r for r in rows if 't' in r]


def key(b):
    return f"{b['reason']}|{b['normal'].get('reason')}|{b['normal'].get('boundary_selection')}"


def stats(rows, name, start):
    rs = [r for r in rows if r['t'] >= start]
    pos = [r for r in rs if r[name]['speed'] > 0]
    reasons = {}
    for r in rs:
        reasons[key(r[name])] = reasons.get(key(r[name]), 0)+1
    return {'from_s': start, 'calls': len(rs), 'positive_speed_calls': len(pos),
            'first_positive_t': pos[0]['t'] if pos else None,
            'speed_range': [min(r[name]['speed'] for r in pos), max(r[name]['speed'] for r in pos)] if pos else None,
            'reason_counts': dict(sorted(reasons.items(), key=lambda kv: -kv[1]))}


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
    return None if a == b else {'path': path, 'recorded': a, 'candidate': b}


def main():
    base, base_rows = load('base')
    cand, cand_rows = load('candidate')
    with zipfile.ZipFile(HERE/'before.zip') as z:
        before = {n: hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
    # before.zip에는 src/arena_autonomy만 있다. 기준선의 arena_vehicle_interface는 작업 트리에서 읽히므로
    # 두 실행의 arena_autonomy 집합을 비교하고, 나머지는 후보 쪽 해시가 작업 트리와 같은지만 기록한다.
    bl = {p: h for p, h in base['loaded_source_sha256'].items() if p.startswith('src/arena_autonomy/')}
    cl = {p: h for p, h in cand['loaded_source_sha256'].items() if p.startswith('src/arena_autonomy/')}
    other_loaded = sorted(p for p in cand['loaded_source_sha256'] if p not in cl)
    checks = {
        'same_input_sha': base['source_sha256_before'] == base['source_sha256_after'] == cand['source_sha256_before']
                          == cand['source_sha256_after'],
        'same_config': base['follow_config'] == cand['follow_config'],
        'standing_wall_reacquire_recorded_false': cand['follow_config']['standing_wall_reacquire'] is False,
        'same_calls_resets': (base['calls_replayed'], base['resets_replayed']) == (cand['calls_replayed'], cand['resets_replayed']),
        'base_full_match': base['mismatches_vs_recorded'] == 0,
        'base_loaded_from_before_zip': all(before.get(p) == h for p, h in bl.items()),
        'same_loaded_file_set': sorted(bl) == sorted(cl),
        'only_two_files_differ': sorted(p for p in cl if cl[p] != bl.get(p)) == sorted(CHANGED),
    }
    changed = [r for r in cand_rows if not r['same_as_recorded']]
    transitions = {}
    for r in changed:
        k = f"{key(r['recorded'])} -> {key(r['replayed'])}"
        transitions[k] = transitions.get(k, 0)+1
    first = changed[0] if changed else None
    if first:
        first = {k: first[k] for k in ('seq', 't', 'measured_speed_mps', 'recorded', 'replayed', 'local_path_calls') if k in first} | {
            'diff': first_diff(changed[0]['comparable_recorded'], changed[0]['comparable_replayed'])}
    # 43.4초 이후 local_path 표현별 진단 유무. 후보에서 일반 곡선(wall_model None) 성공 경로에 필드가 있는지 본다.
    def representation(rows):
        out = {}
        for r in rows:
            for c in r.get('local_path_calls', []):
                if c['reason'] != 'local_path':
                    continue
                k = f"{c['wall_model']}|field={c['has_path_anchor_error']}"
                out[k] = out.get(k, 0)+1
        return out
    general_errors = [c['path_anchor_error_m'] for r in cand_rows for c in r.get('local_path_calls', [])
                      if c['reason'] == 'local_path' and c['wall_model'] is None and r['t'] >= TRANSITION_S]
    summary = {
        'checks': checks,
        'base_mismatches_vs_recorded': base['mismatches_vs_recorded'],
        'candidate_changed_calls_vs_recorded': len(changed),
        'candidate_first_change': first,
        'candidate_change_transitions': dict(sorted(transitions.items(), key=lambda kv: -kv[1])),
        'stall_43_82': {'recorded': stats(cand_rows, 'recorded', STALL_S), 'candidate': stats(cand_rows, 'replayed', STALL_S)},
        'transition_44_02': {'recorded': stats(cand_rows, 'recorded', TRANSITION_S),
                             'candidate': stats(cand_rows, 'replayed', TRANSITION_S)},
        'local_path_representation_after_43_4': {'base': representation(base_rows), 'candidate': representation(cand_rows)},
        'candidate_general_curve_anchor_error_after_44_02': {
            'count': len(general_errors), 'le_0_12': sum(e <= .12 for e in general_errors),
            'min': min(general_errors) if general_errors else None, 'max': max(general_errors) if general_errors else None},
        'changed_files': {p: {'before': bl[p], 'after': cl[p]} for p in CHANGED},
        'arena_autonomy_loaded_files': len(cl), 'other_loaded_from_work_tree': other_loaded,
        'scope': 'stored decision inputs, recorded config; positive command is not proof of closed-loop restart; no Gazebo',
    }
    (HERE/'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({k: summary[k] for k in ('checks', 'base_mismatches_vs_recorded', 'candidate_changed_calls_vs_recorded',
                                              'candidate_change_transitions', 'stall_43_82', 'transition_44_02',
                                              'local_path_representation_after_43_4',
                                              'candidate_general_curve_anchor_error_after_44_02')}, ensure_ascii=False, indent=1))
    print(json.dumps(summary['candidate_first_change'], ensure_ascii=False)[:4000])
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
