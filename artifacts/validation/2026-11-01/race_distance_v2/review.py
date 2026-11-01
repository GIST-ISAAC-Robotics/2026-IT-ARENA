"""완료된 300초 시험의 주행 증거와 기록기 실패를 분리한다. 원시 판정은 수정하지 않는다."""
import importlib.util
import json
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

HERE = Path(__file__).resolve().parent
RUN = HERE / 'run1'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def review():
    report = json.loads((RUN / 'report.json').read_bytes())
    dense = load_module('race_distance_dense', HERE.parent / 'mixed_course_v1/dense_progress.py').audit(RUN)
    green = report['green_observed_sim_s']
    end = report['case_window']['end_sim_s']
    lap = report['route_check']['lap_length_m']
    rows = [json.loads(line) for line in (RUN / 'trajectory.jsonl').read_bytes().splitlines()]
    rows = [r for r in rows if green <= r['t'] <= end]
    if not rows or any(a['t'] >= b['t'] for a, b in zip(rows, rows[1:])):
        raise ValueError('ordered trajectory required')
    laps = []
    previous = 0.
    for n in range(1, 6):
        for a, b in zip(rows, rows[1:]):
            if a['progress_m'] < n * lap <= b['progress_m']:
                cross = a['t'] - green + (b['t'] - a['t']) * (n * lap - a['progress_m']) / (b['progress_m'] - a['progress_m'])
                laps.append(dict(lap=n, elapsed_from_green_s=cross, duration_s=cross - previous))
                previous = cross
                break
    with ZipFile(RUN / 'source_snapshot.zip') as archive:
        mismatches = [name for name, digest in report['source_sha256'].items()
                      if name not in archive.namelist() or sha256(archive.read(name)).hexdigest() != digest]
    contact = report['additional_actors']['combined_contacts']
    return dict(
        scope='한 번의 각본 교통 주행. 평가기 거리 기준 5랩이며 차량 자체 랩 판정/실차/독립 5대 경기가 아니다. 녹화 및 원시 실패는 소급 통과시키지 않는다.',
        report_sha256=sha256((RUN / 'report.json').read_bytes()).hexdigest(),
        raw_run_valid=report['run_valid'], raw_passed=report['passed'],
        shutdown_clean=report['shutdown_clean'], harness_valid=report['harness_valid'],
        source_unchanged=report['source_unchanged'], snapshot_mismatches=mismatches,
        raw_failed_gates=[k for k, v in report['gates'].items() if not v],
        case_window=report['case_window'], dense_progress=dense,
        lap_length_m=lap, laps=laps, five_lap_distance_reached=len(laps) == 5,
        trajectory_maximum_sample_gap_s=max(b['t'] - a['t'] for a, b in zip(rows, rows[1:])),
        peak_sampled_body_mps=max(r['ground_speed_mps'] for r in rows),
        explicit_stop=report['stop_checks'], wall_collision_samples=report['wall_collision_samples'],
        contact_evidence=contact, remaining_test_pids=report['remaining_test_pids'],
        original_evaluation=report['evaluation'])


if __name__ == '__main__':
    value = review()
    with (HERE / 'independent_review.json').open('x', encoding='utf-8') as out:
        json.dump(value, out, ensure_ascii=False, indent=2)
    print(json.dumps({k: value[k] for k in ('raw_run_valid', 'five_lap_distance_reached', 'laps', 'snapshot_mismatches')}, ensure_ascii=False))
    print(json.dumps(value['dense_progress'], ensure_ascii=False))
