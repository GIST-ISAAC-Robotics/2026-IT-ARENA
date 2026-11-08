"""교통 한 랩 run1 저장 판단 입력 0~38초 prefix를 수정 후(local_path.py) 패키지로 재생한다.

기본 OFF 원기록 일치는 traffic_reacquire_replay_v1의 1518/1518 결과를 재사용한다. 그 근거가 유효한지
원본 sha256과 그때 로드한 60개 소스 sha256(이번 수정 전 local_path.py 포함)을 다시 대조한다.
헤더 config 그대로 BypassHandoffFollow, reset 순서 그대로. 저장 센서 대조이며 실제 재출발 증명이 아니다.
"""
import hashlib, json, sys, time, zipfile
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_distance import corner_distance_evidence
from arena_autonomy import local_path as lp

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREV = HERE.parent/'traffic_reacquire_replay_v1'
SOURCE = HERE.parent/'traffic_lap_newcourse_v1'/'two_actor_course_lap'/'run1'/'follow_decisions.jsonl'
LOCAL_PATH = 'src/arena_autonomy/arena_autonomy/local_path.py'
END_S, STALL_S = 38.0, 28.12
GEOMETRY_T = (27.92, 28.22, 33.02)
sys.path.insert(0, str(PREV))
from replay import brief, first_diff  # noqa: E402  같은 요약/차이 함수 재사용


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    prev = json.loads((PREV/'replay_result.json').read_text(encoding='utf-8'))
    with zipfile.ZipFile(HERE/'before.zip') as z:
        before_local_path = hashlib.sha256(z.read(LOCAL_PATH)).hexdigest()
    raw_before = sha(SOURCE)
    reuse = {'previous_baseline_match': prev['baseline_full_comparable_match'] and prev['baseline_mismatch_count'] == 0,
             'previous_calls': prev['calls_replayed'], 'source_sha256_same': raw_before == prev['source_sha256_before'],
             'previous_local_path_sha256': prev['loaded_source_sha256'][LOCAL_PATH],
             'before_zip_local_path_sha256': before_local_path}
    reuse['before_local_path_same'] = reuse['previous_local_path_sha256'] == before_local_path
    controller = mc = cfg = None
    calls = resets = 0
    diffs, rows, budget_calls = [], [], []
    original = lp.local_path

    def traced(*args, **kwargs):
        path, info = original(*args, **kwargs)
        if 'corner_path_budget' in info:
            traced.hits.append(info['corner_path_budget'])
        return path, info
    traced.hits = []
    import arena_autonomy.local_follow as lf
    lf.local_path = traced  # 진단 계수만. 반환값은 그대로 전달한다.
    geometry, targets = [], list(GEOMETRY_T)
    with open(SOURCE, encoding='utf-8') as f:
        for line in f:
            e = json.loads(line)
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
            traced.hits = []
            out = freeze(replay_call(controller, d['input'], mc))
            calls += 1
            rec = freeze(d['output'])
            if not equal_output(comparable_decision(out), comparable_decision(rec)):
                diffs.append({'seq': e['seq'], 't': t, 'diff': first_diff(comparable_decision(rec), comparable_decision(out)),
                              'recorded': brief(rec), 'after': brief(out)})
            if traced.hits:
                budget_calls.append({'seq': e['seq'], 't': t, 'budgets': traced.hits})
            rows.append({'seq': e['seq'], 't': t, 'measured_speed_mps': d['input']['speed_mps'],
                         'recorded': brief(rec), 'after': brief(out), 'corner_path_budget_hits': len(traced.hits)})
            if targets and t >= targets[0]-1e-6:
                targets.pop(0)
                raw, scan, history = restore_call(d['input'], mc)
                sc = motion_scene(scan, history, t, timing_verified=raw['timing_verified'], epoch=raw['epoch'],
                                  rear_half_angle_deg=raw['rear_half_angle_deg'])
                p = np.asarray(sc.occupied_endpoints_xy_m)
                path, info = original(p, 'right', cfg.wall_offset_m, horizon=1.8, piecewise_corner=True,
                                      minimum_radius=cfg.wheelbase_m/np.tan(cfg.max_steering_rad))
                ev = corner_distance_evidence(p, path, 'left', cfg, detailed=True) if path is not None else {}
                geometry.append({'t': t, 'path_ahead_m': None if path is None else lp._path_ahead(path),
                                 'boundary_observed_ahead_m': info.get('boundary_observed_ahead_m'),
                                 'corner_path_budget': info.get('corner_path_budget'),
                                 'evidence_reason': ev.get('reason'), 'supported': ev.get('supported'),
                                 'reference_ahead_m': ev.get('reference_ahead_m')})
    lf.local_path = original
    raw_after = sha(SOURCE)
    call_rows = [r for r in rows if 't' in r]
    stall = [r for r in call_rows if r['t'] >= STALL_S]

    def stats(key, rs):
        pos = [r for r in rs if r[key]['speed'] > 0]
        reasons = {}
        for r in rs:
            k = f"{r[key]['reason']}|{r[key]['normal'].get('reason')}"
            reasons[k] = reasons.get(k, 0)+1
        selections = {}
        for r in rs:
            k = str(r[key]['normal'].get('boundary_selection'))
            selections[k] = selections.get(k, 0)+1
        return {'calls': len(rs), 'positive_speed_calls': len(pos), 'first_positive_t': pos[0]['t'] if pos else None,
                'max_speed_command': max((r[key]['speed'] for r in rs), default=None),
                'nonzero_steer_calls': sum(1 for r in rs if r[key]['steer'] != 0),
                'reason_counts': reasons, 'boundary_selection_counts': selections}
    srcs = sorted({Path(m.__file__).resolve() for m in list(sys.modules.values())
                   if getattr(m, '__file__', None) and str(Path(m.__file__).resolve()).startswith(str(ROOT/'src'))})
    loaded = {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in srcs}
    changed = sorted(k for k, v in loaded.items() if prev['loaded_source_sha256'].get(k) != v)
    with zipfile.ZipFile(HERE/'after_loaded_source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for p in srcs:
            z.write(p, str(p.relative_to(ROOT)).replace('\\', '/'))
    reuse['loaded_sources_changed_vs_previous'] = changed
    reuse['valid'] = (reuse['previous_baseline_match'] and reuse['source_sha256_same'] and reuse['before_local_path_same']
                      and changed == [LOCAL_PATH] and set(loaded) == set(prev['loaded_source_sha256']))
    result = {
        'source': str(SOURCE.relative_to(ROOT)).replace('\\', '/'), 'source_sha256_before': raw_before,
        'source_sha256_after': raw_after, 'source_unchanged': raw_before == raw_after,
        'baseline_reuse': reuse, 'prefix_end_s': END_S, 'calls_replayed': calls, 'resets_replayed': resets,
        'after_vs_recorded_difference_count': len(diffs), 'after_vs_recorded_first10': diffs[:10],
        'corner_path_budget_call_count': len(budget_calls), 'corner_path_budget_first10': budget_calls[:10],
        'stall_window': {'from_s': STALL_S, 'to_s': END_S, 'recorded': stats('recorded', stall), 'after': stats('after', stall)},
        'geometry_after_horizon_1_8': geometry,
        'after_local_path_sha256': loaded[LOCAL_PATH], 'loaded_source_sha256': loaded,
        'scope': 'stored decision inputs only; positive command is not proof of actual restart; no Gazebo',
    }
    (HERE/'replay_after_result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    with open(HERE/'replay_after_rows.jsonl', 'w', encoding='utf-8', newline='\n') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False)+'\n')
    print(json.dumps({k: result[k] for k in ('calls_replayed', 'resets_replayed', 'source_unchanged',
                                              'after_vs_recorded_difference_count', 'corner_path_budget_call_count')}))
    print(json.dumps(reuse))
    print(json.dumps(result['stall_window'], ensure_ascii=False))
    print(json.dumps(geometry))
    print(json.dumps(diffs[:3], ensure_ascii=False)[:4000])
    return 0 if reuse['valid'] and raw_before == raw_after else 1


if __name__ == '__main__':
    sys.exit(main())
