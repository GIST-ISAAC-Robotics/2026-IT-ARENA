"""r43 before/after 저장 기록 재생 비교·시간 측정·격자 생성 전수 대조. ROS·Gazebo·Jetson 미사용.

사용
  python3 -I -B replay_r43.py prepare <before_tree>          before.zip 3파일 + 현재 나머지 소스로 이전 트리 생성
  python3 -I -B replay_r43.py run <repo|before> <label> [<before_tree>]   세 기록 재생(계측 없는 기준 시간)
  python3 -I -B replay_r43.py capture <label>                현재 코드 재생 중 ObservationGrid·threat_view·scene_from_scan 전수 대조
  python3 -I -B replay_r43.py compare                        결과 비교 요약(summary.json)
재생 루프는 r42 bundle_parity.replay_once를 그대로 import한다(같은 헤더·reset·footer 검사, 판단당 timed_call 1회).
"""
from hashlib import sha256
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
import zipfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
PKG = 'src/arena_autonomy/arena_autonomy'
CHANGED = ('local_scene.py', 'lateral_intent.py', 'pass_release_trial.py')
TRACES = {
    'r25_solo': 'artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1/follow_decisions.jsonl',
    'r41_corner': 'artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl',
    'recessed_pass_050': 'artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl',
}
TRACE_SHA = {
    'r25_solo': '11a690d44ea4f7fc5a55eb41f5686b39b50ea448bbf8ff0a0938a06bfde46c60',
    'r41_corner': '43bdc1496f0d8806b23266c06095f2051bb835dc7dbe46756f680df056a86555',
    'recessed_pass_050': '8edcf63fce24cd7657960d03de17d4e572f03fa457a861129dfd92b53d675f72',
}
R42 = REPO/'artifacts/validation/2026-11-08/jetson_bundle_refresh_v1'
RAW = HERE/'raw'
if os.environ.get('R43_SMOKE') == '1':
    # 하네스 점검 전용: r42 v3 파생 픽스처(앞 100판단). 결과는 raw_smoke에만 쓴다.
    RAW = HERE/'raw_smoke'
    TRACES = {'r25_solo': str((R42/'v3/fixtures/r25_solo_first100.jsonl').relative_to(REPO)),
              'r41_corner': str((R42/'v3/fixtures/r41_handoff_first100.jsonl').relative_to(REPO))}
    TRACE_SHA = {'r25_solo': '79277ed11e2a10dd9ad901f31cbfd927e53e9f273c651f5b9aadc346499ed5ea',
                 'r41_corner': 'e94529180325ff91b89dfccf88a2ceb0034a48660543f533718d0a3ce1e32454'}


def file_sha(path):
    value = sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            value.update(chunk)
    return value.hexdigest()


def write_new(path, data):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=1)


def prepare(tree):
    tree = Path(tree)
    target = tree/'arena_autonomy'
    if tree.exists():
        shutil.rmtree(tree)
    shutil.copytree(REPO/PKG, target, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    with zipfile.ZipFile(HERE/'before.zip') as archive:
        for name in CHANGED:
            (target/name).write_bytes(archive.read(f'{PKG}/{name}'))
    files = {p.relative_to(target).as_posix(): file_sha(p) for p in sorted(target.rglob('*.py'))}
    repo = {p.relative_to(REPO/PKG).as_posix(): file_sha(p) for p in sorted((REPO/PKG).rglob('*.py'))
            if '__pycache__' not in p.parts}
    differs = sorted(k for k in set(files) | set(repo) if files.get(k) != repo.get(k))
    assert differs == sorted(CHANGED), differs
    write_new(RAW/'before_tree.json', dict(tree=str(tree), files=files, differs_from_repo=differs,
                                            before_zip_sha256=file_sha(HERE/'before.zip')))
    print('prepared', tree, len(files), differs)


def setup_path(tree):
    root = REPO/'src/arena_autonomy' if tree is None else Path(tree)
    sys.path[:0] = [str(root), str(REPO/'src/arena_vehicle_interface'), str(REPO/'scripts'), str(R42)]
    return root


def loaded_modules(root):
    out = {}
    for name, module in sorted(sys.modules.items()):
        location = getattr(module, '__file__', None)
        if name == '__main__' or not location:
            continue
        path = Path(location).resolve()
        if path.is_relative_to(REPO) or path.is_relative_to(Path(root).resolve()):
            out[name] = dict(path=str(path), sha256=file_sha(path))
    return out


def environment():
    import numpy
    return dict(python=platform.python_version(), numpy=numpy.__version__, system=platform.system(),
                release=platform.release(), machine=platform.machine(), cpus=os.cpu_count(),
                sys_flags=dict(isolated=sys.flags.isolated, ignore_environment=sys.flags.ignore_environment),
                env_python_ros=sorted(k for k in os.environ if k.startswith(('PYTHON', 'ROS', 'AMENT'))))


def check_sources(root, modules, tree):
    """로드된 arena_autonomy 모듈이 선택한 트리에서 왔는지, 바이트가 기대값(현재/이전)과 같은지."""
    root = Path(root).resolve()
    contract = json.loads((REPO/'config/tests/grid_cost_r43_sha256.json').read_text(encoding='utf-8'))
    problems = []
    for name, row in modules.items():
        if not name.startswith('arena_autonomy'):
            continue
        path = Path(row['path'])
        if not path.is_relative_to(root):
            problems.append(f'{name} outside selected tree')
            continue
        rel = path.relative_to(root/'arena_autonomy').as_posix()
        if rel in CHANGED:
            key = f'{PKG}/{rel}'
            expected = contract['approved_changes'][key]['before' if tree else 'after']
        else:
            expected = file_sha(REPO/PKG/rel)
        if row['sha256'] != expected:
            problems.append(f'{name} digest {row["sha256"][:12]} != {expected[:12]}')
    changed_loaded = sorted(n for n in modules if n.split('.')[-1]+'.py' in CHANGED and n.startswith('arena_autonomy'))
    return dict(ok=not problems, problems=problems, changed_modules_loaded=changed_loaded)


def run(which, label, tree=None):
    tree = None if which == 'repo' else tree
    root = setup_path(tree)
    import profile_follow_trace as pft
    from bundle_parity import replay_once
    reports, commands = {}, {}
    for key, rel in TRACES.items():
        path = REPO/rel
        before = file_sha(path)
        assert before == TRACE_SHA[key], key
        begin = time.perf_counter_ns()
        report, rows = replay_once(pft, path)
        report['replay_wall_s'] = (time.perf_counter_ns()-begin)/1e9
        report['input_unchanged'] = file_sha(path) == before
        reports[key], commands[key] = report, rows
    modules = loaded_modules(root)
    result = dict(label=label, tree=which, root=str(root), environment=environment(), replays=reports,
                  source_check=check_sources(root, modules, tree), modules=modules,
                  scope='offline decision replay on this PC/WSL; not Jetson/ROS deadline or closed-loop driving')
    write_new(RAW/f'result_{label}.json', result)
    write_new(RAW/f'commands_{label}.json', commands)
    print(json.dumps({k: dict(calls=v['calls'], stream=v['decision_stream_sha256'][:16], wall=round(v['replay_wall_s'], 2),
                              p50=round(v['costs']['all']['total_ms']['p50'], 2)) for k, v in reports.items()}),
          result['source_check']['ok'])


# ------------------------------------------------------------------ 전수 대조(계측 실행, 시간은 쓰지 않음)
def capture(label):
    root = setup_path(None)
    sys.path.insert(0, str(REPO/'tests'))
    import numpy as np
    import profile_follow_trace as pft
    from bundle_parity import replay_once
    import arena_autonomy.lateral_intent as li
    import arena_autonomy.local_scene as ls
    import arena_autonomy.pass_release_trial as ps
    import arena_autonomy.scene_motion as sm
    import arena_autonomy.scan_topology as st
    import r43_reference_oracle as ref

    stats = dict(grids=0, grid_mismatch=[], by_caller={}, same_call=0, same_call_guard_fail=[],
                 samples_full=0, samples_kept=0, samples_inside=0, threat_views=0, threat_mismatch=[],
                 threat_light_mutated=0, threat_grid_mutated=0, scenes=0, scene_mismatch=[], scene_errors=0)
    pending = []      # 현재 threat_view 호출 안의 (light, light 사본, grid, grid 사본)
    original_init = li.ObservationGrid.__init__

    def snapshot(obj):
        return (obj.returns.copy(), obj.ages.copy(), obj.compact_returns.copy(),
                json.dumps(obj.return_components, sort_keys=True))

    def same_snapshot(obj, snap):
        now = snapshot(obj)
        return all(np.array_equal(a, b) and a.dtype == b.dtype for a, b in zip(now[:3], snap[:3])) and now[3] == snap[3]

    def counted_samples(grid, scene, cfg):
        rays = [r for r in scene.rays if r.status == 'return']
        if not rays:
            return 0, 0
        o = np.asarray([r.ray_origin_xy_m for r in rays], dtype=float)
        vec = np.asarray([r.hit_xy_m for r in rays], dtype=float)-o
        dist = np.linalg.norm(vec, axis=1)
        free = np.maximum(0., dist-cfg.budget.range_bias_m)
        step = grid.r/2
        counts = np.ceil(free/step).astype(int)
        _, kept = grid._free_sample_window(o, vec, dist, free, counts, step)
        return int(np.sum(counts)), int(np.sum(kept))

    def init(self, scene, cfg, **kwargs):
        token = kwargs.get('_same_call_returns')
        light_snap = snapshot(token[2]) if token is not None else None
        original_init(self, scene, cfg, **kwargs)
        caller = sys._getframe(1).f_code.co_name
        stats['grids'] += 1
        stats['by_caller'][caller] = stats['by_caller'].get(caller, 0)+1
        old = ref.reference_grid(scene, cfg)
        a, b = ref.grid_fields(self), ref.grid_fields(old)
        bad = [k for k in ('state', 'returns', 'ages', 'compact', 'observed')
               if not (getattr(a, k).dtype == getattr(b, k).dtype and np.array_equal(getattr(a, k), getattr(b, k)))]
        if a.components != b.components or a.dims != b.dims:
            bad.append('components/dims')
        if bad:
            stats['grid_mismatch'].append(dict(index=stats['grids'], fields=bad))
        full, kept = counted_samples(self, scene, cfg)
        stats['samples_full'] += full
        stats['samples_kept'] += kept
        stats['samples_inside'] += getattr(old, 'reference_inside_samples', 0)
        if token is not None:
            stats['same_call'] += 1
            light = token[2]
            ok = (token[0] is scene and token[1] is cfg and type(light).__name__ == 'ReturnGrid'
                  and not np.shares_memory(self.compact_returns, light.compact_returns)
                  and self.return_components is not light.return_components
                  and all(x is not y for x, y in zip(self.return_components, light.return_components))
                  and same_snapshot(light, light_snap))
            if not ok:
                stats['same_call_guard_fail'].append(stats['grids'])
            pending.append((light, light_snap, self, snapshot(self)))

    original_view = ps.threat_view

    def threat_view(scene, path_points, cfg):
        pending.clear()
        out = original_view(scene, path_points, cfg)
        stats['threat_views'] += 1
        for light, light_snap, grid, grid_snap in pending:
            stats['threat_light_mutated'] += not same_snapshot(light, light_snap)
            stats['threat_grid_mutated'] += not same_snapshot(grid, grid_snap)
        pending.clear()
        old = ref.threat_view(scene, path_points, cfg)
        strip = lambda v: None if v is None else {k: x for k, x in v.items() if k != 'frame'}
        if strip(out) != strip(old):
            stats['threat_mismatch'].append(stats['threat_views'])
        return out

    original_scan = ls.scene_from_scan

    def scene_from_scan(*args, **kwargs):
        stats['scenes'] += 1
        try:
            new = ('ok', original_scan(*args, **kwargs))
        except Exception as error:
            new = (type(error), str(error))
        try:
            old = ('ok', ref.scene_from_scan(*args, **kwargs))
        except Exception as error:
            old = (type(error), str(error))
        if new != old:
            stats['scene_mismatch'].append(stats['scenes'])
        if new[0] != 'ok':
            stats['scene_errors'] += 1
            raise new[0](new[1])
        return new[1]

    li.ObservationGrid.__init__ = init
    ps.threat_view = threat_view
    ls.scene_from_scan = sm.scene_from_scan = st.scene_from_scan = scene_from_scan
    reports, commands, per_trace = {}, {}, {}
    for key, rel in TRACES.items():
        path = REPO/rel
        assert file_sha(path) == TRACE_SHA[key]
        mark = {k: (len(v) if isinstance(v, list) else (dict(v) if isinstance(v, dict) else v)) for k, v in stats.items()}
        report, rows = replay_once(pft, path)
        reports[key], commands[key] = report, rows
        per_trace[key] = {k: (len(v)-mark[k] if isinstance(v, list) else
                              ({c: n-mark[k].get(c, 0) for c, n in v.items()} if isinstance(v, dict) else v-mark[k]))
                          for k, v in stats.items()}
    modules = loaded_modules(root)
    result = dict(label=label, environment=environment(), per_trace=per_trace,
                  totals={k: (len(v) if isinstance(v, list) else v) for k, v in stats.items()},
                  first_failures={k: v[:20] for k, v in stats.items() if isinstance(v, list)},
                  replays=reports, source_check=check_sources(root, modules, None), modules=modules,
                  note='계측 실행. 기준 오라클을 함께 계산하므로 시간은 쓰지 않는다.')
    write_new(RAW/f'result_{label}.json', result)
    write_new(RAW/f'commands_{label}.json', commands)
    print(json.dumps(result['totals']), result['source_check']['ok'])


# ------------------------------------------------------------------ 비교
def compare():
    import numpy as np
    raw = RAW
    runs = sorted(p.stem[len('result_'):] for p in raw.glob('result_*.json'))
    results = {r: json.loads((raw/f'result_{r}.json').read_text(encoding='utf-8')) for r in runs}
    commands = {r: json.loads((raw/f'commands_{r}.json').read_text(encoding='utf-8')) for r in runs}
    r42 = json.loads((R42/'commands_repo.json').read_text(encoding='utf-8'))
    r42_by_key = {k: r42[v] for k, v in TRACES.items() if v in r42}
    base = 'before_1'
    equality = {}
    for run in runs:
        rows = {}
        for key in TRACES:
            a, b = commands[base][key], commands[run][key]
            first = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), None)
            rows[key] = dict(calls=len(b), equal_to_before_1=a == b, first_difference_index=first,
                             stream=results[run]['replays'][key]['decision_stream_sha256'],
                             equal_to_r42=(r42_by_key[key] == b) if key in r42_by_key else None,
                             recorded_mismatch_count=results[run]['replays'][key]['recorded_mismatch_count'])
        equality[run] = rows
    timing = {}
    for run in runs:
        if run.startswith('capture'):
            continue
        timing[run] = {key: dict(wall_s=v['replay_wall_s'],
                                 **{f'{stage}_{q}': v['costs']['all'][stage][q]
                                    for stage in ('total_ms', 'decision_ms') for q in ('p50', 'p95', 'maximum', 'over_20ms')})
                       for key, v in results[run]['replays'].items()}
    def mean_of(prefix, key, field):
        values = [timing[r][key][field] for r in timing if r.startswith(prefix)]
        return sum(values)/len(values) if values else None
    speedup = {}
    for key in TRACES:
        row = {}
        for field in ('wall_s', 'total_ms_p50', 'total_ms_p95', 'decision_ms_p50', 'decision_ms_p95', 'total_ms_over_20ms'):
            b, a = mean_of('before', key, field), mean_of('after', key, field)
            row[field] = dict(before_mean=b, after_mean=a, delta=None if a is None or b is None else a-b)
        speedup[key] = row
    summary = dict(runs=runs, base=base, equality=equality, timing=timing, timing_mean_before_after=speedup,
                   source_checks={r: results[r]['source_check'] for r in runs},
                   capture={r: dict(totals=results[r]['totals'], per_trace=results[r]['per_trace'],
                                    first_failures=results[r]['first_failures'])
                            for r in runs if r.startswith('capture')},
                   environment=results[runs[0]]['environment'])
    with ((HERE if RAW.name == 'raw' else RAW)/'summary.json').open('w', encoding='utf-8', newline='\n') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=1)
    print(json.dumps(dict(equality={r: {k: (v['equal_to_before_1'], v['equal_to_r42']) for k, v in rows.items()}
                                    for r, rows in equality.items()},
                          speed={k: {f: (round(v['before_mean'], 3), round(v['after_mean'], 3)) for f, v in row.items()
                                     if v['before_mean'] is not None and v['after_mean'] is not None}
                                 for k, row in speedup.items()}), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'prepare':
        prepare(sys.argv[2])
    elif mode == 'run':
        run(sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv) > 4 else None)
    elif mode == 'capture':
        capture(sys.argv[2])
    elif mode == 'compare':
        compare()
    else:
        raise SystemExit('unknown mode')
