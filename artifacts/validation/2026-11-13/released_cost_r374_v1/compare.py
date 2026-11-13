"""같은 현재 코드/입력 순서의 baseline 대 후보. 원 r353 출력의 재현 성공으로 보지 않는다."""
import argparse
import hashlib
import importlib.util
import json
import platform
from pathlib import Path
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, freeze, comparable_decision
from arena_autonomy.numeric_kernels import configure


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encoded(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()


def comparable(value):
    # 공통 기존 계약: 정확히 지정된 surface fit.processing_ms만 0으로 대체한다.
    # 명령·진단·광선 개수·이유·기한 등의 정밀도를 낮추거나 필드를 지우지 않는다.
    return encoded(comparable_decision(freeze(value)))


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    position = (len(values)-1)*fraction
    lower = int(position)
    upper = min(lower+1, len(values)-1)
    return values[lower]+(values[upper]-values[lower])*(position-lower)


def metrics(rows, mode):
    subset = [r for r in rows if mode is None or r['state'] == mode]
    baseline = [r['baseline_ms'] for r in subset]
    trial = [r['candidate_ms'] for r in subset]
    return dict(n=len(subset), baseline_p50_ms=percentile(baseline, .5),
                candidate_p50_ms=percentile(trial, .5), baseline_p95_ms=percentile(baseline, .95),
                candidate_p95_ms=percentile(trial, .95),
                baseline_total_ms=sum(baseline), candidate_total_ms=sum(trial),
                hit_calls=sum(r['cache']['hits'] > 0 for r in subset),
                hits=sum(r['cache']['hits'] for r in subset))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('python', 'native_v1'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--verify-hits', action='store_true')
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(A):
        raise ValueError('r374 output must stay inside this artifact')
    out.mkdir(parents=True, exist_ok=False)
    trace = A.parent/'final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'
    boundary_path = A.parents[1]/'2026-11-12/terminal_boundary_r278_v1/candidate.py'
    files = sorted((ROOT/'src/arena_autonomy/arena_autonomy').rglob('*.py'))
    files += sorted((ROOT/'src/arena_vehicle_interface/arena_vehicle_interface').rglob('*.py'))
    files += [trace, boundary_path, A/'candidate.py', A/'compare.py']
    manifest = {p.relative_to(ROOT).as_posix(): digest(p) for p in files}
    (out/'input_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    spec = importlib.util.spec_from_file_location('r374_boundary', boundary_path)
    boundary = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(boundary)
    configure(args.backend)
    events = [json.loads(line) for line in trace.open()]
    header = events[0]['data']
    baseline, motion = make_controller(header)
    trial, _ = make_controller(header)
    rows, mismatches, original_mismatches = [], [], []
    began = time.perf_counter()
    error = None
    boundary.install()
    try:
        for event in events[1:]:
            data = event['data']
            if event['kind'] == 'reset':
                replay_reset(baseline, data)
                replay_reset(trial, data)
            elif event['kind'] == 'call':
                outputs, durations, cache_rows = {}, {}, []
                # 첫 실행의 일관된 캐시/열 효과를 한쪽에만 주지 않는다.
                order = ('baseline', 'candidate') if len(rows) % 2 == 0 else ('candidate', 'baseline')
                for mode in order:
                    start = time.perf_counter()
                    if mode == 'baseline':
                        outputs[mode] = replay_call(baseline, data['input'], motion)
                    else:
                        with candidate.enabled(observer=cache_rows.append, verify_hits=args.verify_hits):
                            outputs[mode] = replay_call(trial, data['input'], motion)
                    durations[mode] = (time.perf_counter()-start)*1000
                before, after = comparable(outputs['baseline']), comparable(outputs['candidate'])
                same = before == after
                normal = outputs['baseline'][2].get('normal') or outputs['baseline'][2]
                cache = {key: sum(r[key] for r in cache_rows)
                         for key in ('calls', 'misses', 'hits', 'saved_queries', 'verified_hits')}
                row = dict(seq=event['seq'], reference_s=data['input']['reference_s'],
                           state=normal.get('selector_mode'), exact_equal=same,
                           baseline_ms=durations['baseline'], candidate_ms=durations['candidate'],
                           baseline_sha256=hashlib.sha256(before).hexdigest(),
                           candidate_sha256=hashlib.sha256(after).hexdigest(), cache=cache)
                rows.append(row)
                if comparable(outputs['baseline']) != comparable(data['output']):
                    original_mismatches.append(event['seq'])
                if not same:
                    mismatches.append(event['seq'])
                    (out/'first_mismatch.json').write_text(json.dumps(dict(
                        seq=event['seq'], baseline=freeze(outputs['baseline']),
                        candidate=freeze(outputs['candidate'])), indent=2)+'\n')
                    raise AssertionError('full decision mismatch; stop without timing adoption')
    except BaseException as caught:
        error = type(caught).__name__+': '+str(caught)
        raise
    finally:
        boundary.uninstall()
        drift = [name for name, value in manifest.items() if digest(ROOT/name) != value]
        totals = {key: sum(r['cache'][key] for r in rows)
                  for key in ('calls', 'misses', 'hits', 'saved_queries', 'verified_hits')}
        released = metrics(rows, 'RELEASED')
        reduction = (1-released['candidate_total_ms']/released['baseline_total_ms']
                     if released['baseline_total_ms'] else None)
        report = dict(calls=len(rows), exact_equal=sum(r['exact_equal'] for r in rows),
                      mismatches=mismatches, error=error, changed_sources=drift,
                      backend=args.backend, verify_hits=args.verify_hits, cache=totals,
                      all=metrics(rows, None), released=released,
                      released_total_reduction_fraction=reduction,
                      original_record_full_mismatches=original_mismatches,
                      platform=platform.platform(), elapsed_s=time.perf_counter()-began,
                      performance_measurement_valid=not args.verify_hits,
                      scope='paired current-controller serial replay; no ROS/devices/runtime deadline proof',
                      further_timed_comparison_justified=(not args.verify_hits and not mismatches and
                          not drift and error is None and reduction is not None and reduction >= .05))
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'rows.json').write_text(json.dumps(rows, indent=2)+'\n')
        print(json.dumps(report), flush=True)
    if drift:
        raise AssertionError('source/input drift')
    if not rows or len(rows) != sum(e['kind'] == 'call' for e in events):
        raise AssertionError('incomplete paired replay')


if __name__ == '__main__':
    main()
