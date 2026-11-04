"""원본/후보 패키지를 분리해 저장 판단을 재생한다. 실제 대체 궤적 증명은 아니다."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT/'build/surface_finite_solver_v1'
TRACES = {
    'failed_ray': ROOT/'artifacts/validation/2026-11-02/solo_ray_combination_v1/run1/follow_decisions.jsonl',
    'selected_solo': ROOT/'artifacts/validation/2026-11-03/solo_post_free_v1/run1/follow_decisions.jsonl',
    'irregular_bypass': ROOT/'artifacts/validation/2026-11-03/irregular_actor_v1/run1/follow_decisions.jsonl',
}
TESTS = ('test_partial_surface_alignment.py', 'test_partial_observation.py', 'test_lidar_safety.py',
         'test_surface_reobservation.py', 'test_surface_reobservation_connection.py',
         'test_surface_anchor_probe.py', 'test_anchor_clearance_probe.py',
         'test_small_surface_alignment.py', 'test_small_surface_alignment_geometry.py')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def prepare():
    BASE.mkdir(exist_ok=False)
    files = {}
    for mode in ('original', 'candidate8'):
        for package in ('arena_autonomy', 'arena_vehicle_interface'):
            source = ROOT/'src'/package/package
            for p in source.rglob('*.py'):
                relative = p.relative_to(source.parent)
                data = p.read_bytes()
                if mode == 'candidate8' and relative.as_posix() == 'arena_autonomy/partial_surface_alignment.py':
                    data = (HERE/'candidate8.py').read_bytes()
                target = BASE/mode/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
                files[target.relative_to(BASE).as_posix()] = sha(data)
    with zipfile.ZipFile(HERE/'replay_sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in files:
            archive.write(BASE/name, name)
    registered = dict(files=files, inputs={k: sha(p.read_bytes()) for k, p in TRACES.items()},
                      runtime_source=sha((ROOT/'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py').read_bytes()))
    (HERE/'replay_registered.json').write_text(json.dumps(registered, indent=2), encoding='utf-8')


def modules_are_isolated(mode):
    paths = [Path(m.__file__).resolve() for name, m in sys.modules.items()
             if name.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(m, '__file__', None)]
    if not paths or not all(p.is_relative_to((BASE/mode).resolve()) for p in paths):
        raise ValueError('mixed runtime and isolated modules')
    return len(paths)


def child(mode, label):
    sys.path.insert(0, str(BASE/mode))
    import numpy as np
    from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
    from arena_autonomy.local_follow import FollowConfig, LocalFollow
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
    if label == 'tests':
        import pytest
        code = pytest.main(['-q', *[str(ROOT/'tests'/p) for p in TESTS]])
        modules_are_isolated(mode)
        return code
    source = TRACES[label]
    rows, costs, mismatches, first_diff, footer = [], [], 0, [], None
    seq = 0
    for line in source.open(encoding='utf-8'):
        event = json.loads(line)
        if footer is not None or event['seq'] != seq:
            raise ValueError('recorded event sequence invalid')
        seq += 1
        data = event['data']
        if event['kind'] == 'header':
            cfg, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
            control = (BypassHandoffFollow(cfg, HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
                       if 'execution_config' in data else LocalFollow(cfg))
        elif event['kind'] == 'reset':
            control.reset()
        elif event['kind'] == 'call':
            began = time.perf_counter()
            actual = replay_call(control, data['input'], motion)
            costs.append((time.perf_counter()-began)*1000)
            actual = freeze(actual)
            same = equal_output(comparable_decision(actual), comparable_decision(data['output']))
            mismatches += not same
            if not same and len(first_diff) < 5:
                first_diff.append(dict(seq=event['seq'], reference_s=data['input']['reference_s'],
                                       expected=data['output'][:2], actual=actual[:2]))
            d = actual[2]
            normal = d.get('normal')
            part = ((normal if isinstance(normal, dict) else d).get('partial_observation') or {})
            fit = part.get('surface_correspondence') or {}
            rows.append(dict(seq=event['seq'], t=data['input']['reference_s'], dt=data['input']['dt_s'],
                             speed=actual[0], steer=actual[1], reason=d.get('reason'), state=d.get('handoff_state'),
                             partial_reason=part.get('reason'), fit_accepted=fit.get('accepted'), fit_reason=fit.get('reason')))
        elif event['kind'] == 'footer':
            footer = data
        else:
            raise ValueError('unrecognized event')
    if (not rows or footer is None or any(footer.get(k) != 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
            or footer.get('worker_alive') is not False or footer.get('written') != seq-1 or footer.get('accepted') != seq-1):
        raise ValueError('incomplete source trace')
    isolated = modules_are_isolated(mode)
    result = dict(mode=mode, trace=label, input_sha256=sha(source.read_bytes()), calls=len(rows),
                  recorded_decision_mismatches=mismatches, first_differences=first_diff, isolated_modules=isolated,
                  zero_command_sample_time_s=sum(r['dt'] for r in rows if r['speed'] <= 1e-9),
                  decision_cost_ms=dict(median=float(np.median(costs)), p95=float(np.percentile(costs, 95)), maximum=max(costs)),
                  scope='판단 함수 저장 입력 재생. 출력 차이 뒤 실제 센서/궤적을 생성한 폐루프가 아니며 독립 보호층 전체 재생도 아니다.')
    (HERE/f'{label}_{mode}_decisions.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
    (HERE/f'{label}_{mode}.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--child', nargs=2)
    options = parser.parse_args()
    if options.child:
        return child(*options.child)
    prepare()
    jobs = [('candidate8', 'tests'), *[(mode, label) for label in TRACES for mode in ('original', 'candidate8')]]
    for mode, label in jobs:
        with (HERE/f'{label}_{mode}.log').open('x') as stream:
            result = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()), '--child', mode, label],
                                    stdout=stream, stderr=subprocess.STDOUT, cwd=ROOT)
        (HERE/f'{label}_{mode}.exit').write_text(f'ACTUAL_EXIT_CODE={result.returncode}\n', encoding='utf-8')
        if result.returncode:
            return result.returncode
    expected = json.loads((HERE/'replay_registered.json').read_bytes())['runtime_source']
    actual = sha((ROOT/'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py').read_bytes())
    if actual != expected:
        raise ValueError('runtime source changed during isolated comparison')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
