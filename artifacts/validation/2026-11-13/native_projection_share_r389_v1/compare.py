"""동일399입력 전체 판단·부분관측/선택기 상태 비교와 별도 nonprofile 비용 비교."""
import argparse
from contextlib import nullcontext
from dataclasses import fields, is_dataclass
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
import time

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import comparable_decision, freeze, replay_call, replay_reset
from arena_autonomy.numeric_kernels import configure


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encoded(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')).encode('utf-8')


def comparable(value):
    return encoded(comparable_decision(freeze(value)))


def state_value(value):
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float:
        return {'float': value.hex()}
    if isinstance(value, np.ndarray):
        if value.dtype.hasobject:
            raise TypeError('object array is not supported in deterministic state')
        return {'array_dtype': value.dtype.str, 'shape': list(value.shape), 'bytes': value.tobytes().hex()}
    if isinstance(value, np.generic):
        return {'scalar_dtype': value.dtype.str, 'bytes': value.tobytes().hex()}
    if isinstance(value, dict):
        # tuple 키(point_times 등)도 타입/값을 유지한다. 문자열 변환으로 합치지 않는다.
        return {'dict': sorted(([state_value(k), state_value(v)] for k, v in value.items()), key=encoded)}
    if isinstance(value, (set, frozenset)):
        return {type(value).__name__: sorted((state_value(v) for v in value), key=encoded)}
    if isinstance(value, (list, tuple)):
        return {type(value).__name__: [state_value(v) for v in value]}
    if is_dataclass(value):
        return {'type': type(value).__module__+'.'+type(value).__qualname__,
                'fields': state_value({f.name: getattr(value, f.name) for f in fields(value)})}
    if hasattr(value, '__dict__'):
        return {'type': type(value).__module__+'.'+type(value).__qualname__, 'attributes': state_value(vars(value))}
    raise TypeError('unsupported state value: '+type(value).__name__)


def controller_state(follow):
    normal = getattr(follow, 'normal', follow)
    policy = vars(normal.partial_policy).copy()
    cache = policy.get('surface_cache')
    # 기존 comparable_decision과 같은 표면 정합 wall-clock 진단의 저장 사본만 정규화.
    # 다른 시각/processing_ms 이름의 필드를 재귀적으로 지우지 않는다.
    if cache is not None and len(cache) == 3 and isinstance(cache[2], dict) and 'processing_ms' in cache[2]:
        value = cache[2]['processing_ms']
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError('invalid cached surface processing cost')
        policy['surface_cache'] = (cache[0], cache[1], {**cache[2], 'processing_ms': 0.})
    return state_value(dict(partial_policy=policy, partial_policy_type=type(normal.partial_policy).__qualname__,
                           partial_epoch=normal.partial_epoch, selector=normal.pass_selector))


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    position = (len(values)-1)*fraction
    low, high = int(position), min(int(position)+1, len(values)-1)
    return values[low]+(values[high]-values[low])*(position-low)


def metrics(rows, state=None):
    subset = [row for row in rows if state is None or row['selector_mode'] == state]
    result = dict(calls=len(subset))
    for mode in ('baseline', 'candidate'):
        values = [row[mode+'_ms'] for row in subset]
        result[mode] = dict(total_ms=sum(values), p50_ms=percentile(values, .5), p95_ms=percentile(values, .95))
    total = result['baseline']['total_ms']
    result['total_reduction_fraction'] = 1-result['candidate']['total_ms']/total if total else None
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('python', 'native_v1'), required=True)
    parser.add_argument('--verify-points', action='store_true')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(A):
        raise ValueError('output must stay in r389 artifact')
    out.mkdir(parents=True, exist_ok=False)
    trace = A.parent/'final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'
    boundary_path = A.parents[1]/'2026-11-12/terminal_boundary_r278_v1/candidate.py'
    files = sorted((ROOT/'src/arena_autonomy/arena_autonomy').rglob('*.py'))
    files += sorted((ROOT/'src/arena_vehicle_interface/arena_vehicle_interface').rglob('*.py'))
    files += [trace, boundary_path, A/'candidate.py', A/'compare.py', A/'staged/native_projection.py']
    manifest = {p.relative_to(ROOT).as_posix(): digest(p) for p in files}
    (out/'input_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    spec = importlib.util.spec_from_file_location('r389_boundary', boundary_path)
    boundary = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(boundary)
    configure(args.backend)
    events = [json.loads(line) for line in trace.read_text(encoding='utf-8').splitlines()]
    baseline, motion = make_controller(events[0]['data'])
    trial, _ = make_controller(events[0]['data'])
    rows, failures, original_mismatches = [], [], []
    resets, error = 0, None
    began_all = time.perf_counter()
    boundary.install()
    try:
        for event in events[1:]:
            data = event['data']
            if event['kind'] == 'reset':
                replay_reset(baseline, data); replay_reset(trial, data)
                resets += 1
                if encoded(controller_state(baseline)) != encoded(controller_state(trial)):
                    raise AssertionError('state differs after reset')
            elif event['kind'] == 'call':
                outputs, states, durations, stats = {}, {}, {}, {}
                order = ('baseline', 'candidate') if len(rows) % 2 == 0 else ('candidate', 'baseline')
                for mode in order:
                    follow = baseline if mode == 'baseline' else trial
                    context = nullcontext({}) if mode == 'baseline' else candidate.enabled(verify=args.verify_points)
                    # offline 함수 설치/복구와 상태 직렬화는 판단 비용에서 제외한다.
                    # 실제 기하 공유 객체 생성·입력 대조·반환 배열 생성 비용은 포함한다.
                    with context as stats[mode]:
                        began = time.perf_counter()
                        outputs[mode] = replay_call(follow, data['input'], motion)
                        durations[mode] = (time.perf_counter()-began)*1000
                    states[mode] = controller_state(follow)
                before, after = comparable(outputs['baseline']), comparable(outputs['candidate'])
                same_output, same_state = before == after, encoded(states['baseline']) == encoded(states['candidate'])
                normal = outputs['baseline'][2].get('normal') or outputs['baseline'][2]
                rows.append(dict(seq=event['seq'], selector_mode=normal.get('selector_mode'),
                                 output_equal=same_output, state_equal=same_state,
                                 baseline_ms=durations['baseline'], candidate_ms=durations['candidate'],
                                 projection=stats['candidate'], baseline_sha256=hashlib.sha256(before).hexdigest(),
                                 candidate_sha256=hashlib.sha256(after).hexdigest()))
                if before != comparable(data['output']):
                    original_mismatches.append(event['seq'])
                if not (same_output and same_state):
                    failures.append(event['seq'])
                    (out/'first_mismatch.json').write_text(json.dumps(dict(seq=event['seq'],
                        outputs=freeze(outputs), states=states), indent=2)+'\n', encoding='utf-8')
                    raise AssertionError('full output or partial/selector state differs; stop')
    except BaseException as caught:
        error = type(caught).__name__+': '+str(caught)
        raise
    finally:
        boundary.uninstall()
        changed = [name for name, sha in manifest.items() if digest(ROOT/name) != sha]
        counts = {key: sum(row['projection'][key] for row in rows)
                  for key in ('scene_calls', 'shareable_scenes', 'points_calls', 'shared_points_calls',
                              'fallback_points_calls', 'verified_points_calls')}
        report = dict(calls=len(rows), resets=resets, output_equal=sum(row['output_equal'] for row in rows),
                      state_equal=sum(row['state_equal'] for row in rows), failures=failures, error=error,
                      changed_sources=changed, projection=counts,
                      calls_with_shared_projection=sum(row['projection']['shared_points_calls'] > 0 for row in rows),
                      original_record_full_mismatches=original_mismatches, all=metrics(rows),
                      released=metrics(rows, 'RELEASED'), backend=args.backend, verify_points=args.verify_points,
                      performance_measurement_valid=not args.verify_points, platform=platform.platform(),
                      elapsed_s=time.perf_counter()-began_all,
                      scope='serial same-input current-controller replay; no ROS/devices/current-stage/deadline claim')
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        (out/'rows.json').write_text(json.dumps(rows, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report), flush=True)
    if changed or len(rows) != 399 or not counts['shared_points_calls']:
        raise AssertionError('source changed, representative399 incomplete, or no shared projection')


if __name__ == '__main__':
    main()
