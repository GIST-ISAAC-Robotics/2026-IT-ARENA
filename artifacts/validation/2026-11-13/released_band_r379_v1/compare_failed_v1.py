"""동일399입력의 전체 판단·선택기 상태·띠/창 사전 비교. 장치/ROS/발행 없음."""
import argparse
from contextlib import contextmanager, nullcontext
from dataclasses import asdict, is_dataclass
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import candidate
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, freeze, comparable_decision
from arena_autonomy.numeric_kernels import configure


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encoded(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()


def comparable(value):
    # 공통 기존 계약의 표면 정합 processing_ms만 정규화한다. 그 밖의 값은 정확 비교한다.
    return encoded(comparable_decision(freeze(value)))


def state_value(value):
    if is_dataclass(value):
        return state_value(asdict(value))
    if isinstance(value, dict):
        return {k: state_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [state_value(v) for v in value]
    if hasattr(value, '__dict__'):
        return state_value(vars(value))
    return freeze(value)


def selector_state(follow):
    normal = getattr(follow, 'normal', follow)
    return state_value(normal.pass_selector)


@contextmanager
def capture_bands(rows):
    original = ps.band_check
    def captured(scene, frame, sign, cfg, sel, longitudinal=None):
        result = original(scene, frame, sign, cfg, sel, longitudinal)
        rows.append(freeze(dict(sign=sign, longitudinal=longitudinal, band=result)))
        return result
    ps.band_check = captured
    try:
        yield
    finally:
        ps.band_check = original


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    position = (len(values)-1)*fraction
    lower, upper = int(position), min(int(position)+1, len(values)-1)
    return values[lower]+(values[upper]-values[lower])*(position-lower)


def metrics(rows, state):
    subset = [r for r in rows if state is None or r['selector_mode'] == state]
    result = dict(n=len(subset), band_calls=sum(r['band_calls'] for r in subset))
    for mode in ('baseline', 'candidate'):
        values = [r[mode+'_ms'] for r in subset]
        result.update({mode+'_p50_ms': percentile(values, .5), mode+'_p95_ms': percentile(values, .95),
                       mode+'_total_ms': sum(values)})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('python', 'native_v1'), required=True)
    parser.add_argument('--verify-arrays', action='store_true')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(A):
        raise ValueError('r379 output must stay in this artifact')
    out.mkdir(parents=True, exist_ok=False)
    trace = A.parent/'final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'
    boundary_path = A.parents[1]/'2026-11-12/terminal_boundary_r278_v1/candidate.py'
    files = sorted((ROOT/'src/arena_autonomy/arena_autonomy').rglob('*.py'))
    files += sorted((ROOT/'src/arena_vehicle_interface/arena_vehicle_interface').rglob('*.py'))
    files += [trace, boundary_path, A/'candidate.py', A/'compare.py']
    manifest = {p.relative_to(ROOT).as_posix(): digest(p) for p in files}
    (out/'input_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    spec = importlib.util.spec_from_file_location('r379_boundary', boundary_path)
    boundary = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(boundary)
    configure(args.backend)
    events = [json.loads(line) for line in trace.open()]
    baseline, motion = make_controller(events[0]['data'])
    trial, _ = make_controller(events[0]['data'])
    rows, failures, original_mismatches = [], [], []
    reset_count = 0
    error = None
    start_all = time.perf_counter()
    boundary.install()
    try:
        for event in events[1:]:
            data = event['data']
            if event['kind'] == 'reset':
                replay_reset(baseline, data)
                replay_reset(trial, data)
                reset_count += 1
                if encoded(selector_state(baseline)) != encoded(selector_state(trial)):
                    raise AssertionError('selector state mismatch after reset')
            elif event['kind'] == 'call':
                outputs, states, bands, durations = {}, {}, {}, {}
                order = ('baseline', 'candidate') if len(rows) % 2 == 0 else ('candidate', 'baseline')
                for mode in order:
                    follow = baseline if mode == 'baseline' else trial
                    bands[mode] = []
                    context = nullcontext() if mode == 'baseline' else candidate.enabled(verify=args.verify_arrays)
                    start = time.perf_counter()
                    with context, capture_bands(bands[mode]):
                        outputs[mode] = replay_call(follow, data['input'], motion)
                    durations[mode] = (time.perf_counter()-start)*1000
                    states[mode] = selector_state(follow)
                before, after = comparable(outputs['baseline']), comparable(outputs['candidate'])
                same_output = before == after
                same_state = encoded(states['baseline']) == encoded(states['candidate'])
                same_band = encoded(bands['baseline']) == encoded(bands['candidate'])
                normal = outputs['baseline'][2].get('normal') or outputs['baseline'][2]
                row = dict(seq=event['seq'], selector_mode=normal.get('selector_mode'),
                           output_equal=same_output, state_equal=same_state, band_equal=same_band,
                           baseline_ms=durations['baseline'], candidate_ms=durations['candidate'],
                           band_calls=len(bands['candidate']),
                           band_samples=sum(b['band']['samples'] for b in bands['candidate']),
                           baseline_sha256=hashlib.sha256(before).hexdigest(),
                           candidate_sha256=hashlib.sha256(after).hexdigest())
                rows.append(row)
                if before != comparable(data['output']):
                    original_mismatches.append(event['seq'])
                if not (same_output and same_state and same_band):
                    failures.append(event['seq'])
                    (out/'first_mismatch.json').write_text(json.dumps(dict(
                        seq=event['seq'], outputs=freeze(outputs), states=states, bands=bands), indent=2)+'\n')
                    raise AssertionError('full output/state/band mismatch; stop')
    except BaseException as caught:
        error = type(caught).__name__+': '+str(caught)
        raise
    finally:
        boundary.uninstall()
        changed = [name for name, value in manifest.items() if digest(ROOT/name) != value]
        released = metrics(rows, 'RELEASED')
        reduction = (1-released['candidate_total_ms']/released['baseline_total_ms']
                     if released['baseline_total_ms'] else None)
        report = dict(calls=len(rows), resets=reset_count,
                      output_equal=sum(r['output_equal'] for r in rows),
                      state_equal=sum(r['state_equal'] for r in rows),
                      band_equal=sum(r['band_equal'] for r in rows),
                      band_calls=sum(r['band_calls'] for r in rows),
                      band_samples=sum(r['band_samples'] for r in rows),
                      failures=failures, error=error, changed_sources=changed,
                      backend=args.backend, verify_arrays=args.verify_arrays,
                      performance_measurement_valid=not args.verify_arrays,
                      all=metrics(rows, None), released=released,
                      released_total_reduction_fraction=reduction,
                      original_record_full_mismatches=original_mismatches,
                      platform=platform.platform(), elapsed_s=time.perf_counter()-start_all,
                      scope='paired current-controller serial replay; no current stage/ROS/devices/deadline proof')
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'rows.json').write_text(json.dumps(rows, indent=2)+'\n')
        print(json.dumps(report), flush=True)
    if changed or len(rows) != sum(e['kind'] == 'call' for e in events):
        raise AssertionError('changed source or incomplete comparison')


if __name__ == '__main__':
    main()
