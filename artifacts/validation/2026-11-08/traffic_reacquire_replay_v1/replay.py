"""교통 한 랩 run1 저장 판단 입력을 0~38초 prefix로 재생해 기본값과 standing_wall_reacquire=True를 비교한다.

새 장애물/지도/정답 입력이나 임계값 변경 없음. 헤더 config 그대로, 옵션 하나만 dataclasses.replace.
reset 이벤트를 같은 순서로 두 제어기에 적용한다. 저장 입력 재생이며 실제 재출발 증명이 아니다.
"""
import dataclasses, hashlib, json, sys, time, zipfile
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE = HERE.parent/'traffic_lap_newcourse_v1'/'two_actor_course_lap'/'run1'/'follow_decisions.jsonl'
END_S = 38.0
STALL_S = 28.12


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


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
            return {'path': path, 'only_baseline': sorted(set(a)-set(b)), 'only_on': sorted(set(b)-set(a))}
        for k in a:
            d = first_diff(a[k], b[k], f'{path}/{k}')
            if d:
                return d
        return None
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return {'path': path, 'len_baseline': len(a), 'len_on': len(b)}
        for i, (x, y) in enumerate(zip(a, b)):
            d = first_diff(x, y, f'{path}/{i}')
            if d:
                return d
        return None
    return None if equal_output(a, b) else {'path': path, 'baseline': a, 'on': b}


def main():
    raw_sha_before = sha(SOURCE)
    with open(SOURCE, 'rb') as f:
        f.seek(-4096, 2)
        footer = json.loads(f.read().splitlines()[-1])
    assert footer['kind'] == 'footer', footer
    fd = footer['data']
    assert fd['capture_errors'] == 0 and fd['dropped'] == 0 and fd['failed'] == 0 and fd['pending'] == 0, fd
    controllers = mc = None
    expected_seq = 0
    calls = resets = 0
    mismatches, first_on_diff, rows = [], None, []
    costs = []
    with open(SOURCE, encoding='utf-8') as f:
        for line in f:
            e = json.loads(line)
            assert e['seq'] == expected_seq, (e['seq'], expected_seq)
            expected_seq += 1
            if e['kind'] == 'header':
                h = e['data']
                mc = MotionConfig(**h['motion_config'])
                base_cfg = FollowConfig(**h['follow_config'])
                assert base_cfg.standing_wall_reacquire is False
                on_cfg = dataclasses.replace(base_cfg, standing_wall_reacquire=True)
                ex = HandoffExecutionConfig.from_mapping(h['execution_config'])
                controllers = {'baseline': BypassHandoffFollow(base_cfg, ex, allow_simulation_trial=True),
                               'on': BypassHandoffFollow(on_cfg, ex, allow_simulation_trial=True)}
                continue
            if e['kind'] == 'reset':
                for c in controllers.values():
                    c.reset()
                resets += 1
                rows.append({'seq': e['seq'], 'reset': e['data'].get('reason')})
                continue
            if e['kind'] != 'call':
                break
            d = e['data']
            t = d['input']['reference_s']
            if t > END_S:
                break
            start = time.perf_counter()
            base = freeze(replay_call(controllers['baseline'], d['input'], mc))
            costs.append(time.perf_counter()-start)
            on = freeze(replay_call(controllers['on'], d['input'], mc))
            calls += 1
            if not equal_output(comparable_decision(base), comparable_decision(d['output'])):
                mismatches.append({'seq': e['seq'], 't': t,
                                   'diff': first_diff(comparable_decision(d['output']), comparable_decision(base))})
            if first_on_diff is None and not equal_output(comparable_decision(on), comparable_decision(base)):
                first_on_diff = {'seq': e['seq'], 't': t, 'diff': first_diff(comparable_decision(base), comparable_decision(on)),
                                 'baseline': brief(base), 'on': brief(on)}
            rows.append({'seq': e['seq'], 't': t, 'measured_speed_mps': d['input']['speed_mps'],
                         'recorded': brief(freeze(d['output'])), 'baseline': brief(base), 'on': brief(on)})
    raw_sha_after = sha(SOURCE)
    calls_rows = [r for r in rows if 't' in r]
    stall = [r for r in calls_rows if r['t'] >= STALL_S]

    def stats(key, rs):
        pos = [r for r in rs if r[key]['speed'] > 0]
        reasons = {}
        for r in rs:
            k = f"{r[key]['reason']}|{r[key]['normal'].get('reason')}"
            reasons[k] = reasons.get(k, 0)+1
        return {'calls': len(rs), 'positive_speed_calls': len(pos),
                'first_positive_t': pos[0]['t'] if pos else None,
                'nonzero_steer_calls': sum(1 for r in rs if r[key]['steer'] != 0),
                'reason_counts': reasons}
    srcs = sorted({Path(m.__file__).resolve() for m in list(sys.modules.values())
                   if getattr(m, '__file__', None) and str(Path(m.__file__).resolve()).startswith(str(ROOT/'src'))})
    src_hashes = {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in srcs}
    with zipfile.ZipFile(HERE/'loaded_source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for p in srcs:
            z.write(p, str(p.relative_to(ROOT)))
    # 실행 당시 run1/source_snapshot.zip과 현재 로드 소스 대조(읽기 전용)
    snap = SOURCE.parent/'source_snapshot.zip'
    snapshot_compare = {}
    with zipfile.ZipFile(snap) as z:
        names = z.namelist()
        for rel, digest in src_hashes.items():
            match = [n for n in names if n.endswith(rel.split('src/', 1)[-1])]
            snapshot_compare[rel] = ('missing' if not match else
                                     'same' if hashlib.sha256(z.read(match[0])).hexdigest() == digest else 'different')
    result = {
        'source': str(SOURCE.relative_to(ROOT)).replace('\\', '/'),
        'source_sha256_before': raw_sha_before, 'source_sha256_after': raw_sha_after,
        'source_unchanged': raw_sha_before == raw_sha_after, 'footer': fd,
        'prefix_end_s': END_S, 'calls_replayed': calls, 'resets_replayed': resets,
        'baseline_full_comparable_match': not mismatches, 'baseline_mismatch_count': len(mismatches),
        'baseline_mismatches_first5': mismatches[:5],
        'on_first_difference': first_on_diff,
        'stall_window': {'from_s': STALL_S, 'to_s': END_S, 'recorded': stats('recorded', stall),
                         'baseline': stats('baseline', stall), 'on': stats('on', stall)},
        'baseline_wall_seconds_per_call': {'median': sorted(costs)[len(costs)//2], 'max': max(costs)},
        'loaded_source_sha256': src_hashes, 'run_snapshot_compare': snapshot_compare,
        'scope': 'stored decision inputs only; positive command is not proof of actual restart; no Gazebo',
    }
    (HERE/'replay_result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    with open(HERE/'replay_rows.jsonl', 'w', encoding='utf-8', newline='\n') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False)+'\n')
    print(json.dumps({k: result[k] for k in ('calls_replayed', 'resets_replayed', 'baseline_full_comparable_match',
                                              'baseline_mismatch_count', 'source_unchanged')}))
    print(json.dumps(result['stall_window'], ensure_ascii=False))
    print(json.dumps(first_on_diff, ensure_ascii=False)[:3000])
    return 0 if not mismatches and raw_sha_before == raw_sha_after else 1


if __name__ == '__main__':
    sys.exit(main())
