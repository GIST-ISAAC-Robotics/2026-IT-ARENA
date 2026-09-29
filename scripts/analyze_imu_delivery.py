#!/usr/bin/env python3
"""완전한 콜백 진단에서 IMU 누락·공통/개별 손실·시간 간격을 구분한다."""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
from pathlib import Path

from analyze_wheel_delivery import distribution
from replay_imu_probe import read_result as read_probe_result


def loss_runs(expected, missing):
    """원본 순번의 연속 누락. 첫/끝에는 보간 가능한 양끝 표본이 없을 수 있다."""
    groups, current = [], []
    for index, stamp in enumerate(expected):
        if stamp in missing:
            current.append(index)
        elif current:
            groups.append(current)
            current = []
    if current:
        groups.append(current)
    result = []
    for group in groups:
        first, last = group[0], group[-1]
        before = expected[first-1] if first else None
        after = expected[last+1] if last+1 < len(expected) else None
        result.append(dict(count=len(group), first_source_ns=expected[first],
            last_source_ns=expected[last], before_source_ns=before, after_source_ns=after,
            bracketing_source_gap_ms=(after-before)/1e6 if before is not None and after is not None else None))
    return result


def analyze_rows(expected, rows, report):
    if len(expected) != len(set(expected)) or expected != sorted(expected):
        raise ValueError('Unique ordered source timestamps required; no ambiguous matching')
    source = set(expected)
    window = report['active_window_continuity']
    start, end = window['reference_start_ns'], window['reference_end_ns']
    result, losses = {}, {}
    for name in ('local_pursuit', 'lidar_safety'):
        key = name+'/imu'
        observed = sorted((r for r in rows if r['node'] == name and r['callback'] == 'imu'),
                          key=lambda r: r['begin_monotonic_ns'])
        coverage = report['diagnostic_sequence_coverage'][key]
        delivery = report['input_callback_delivery'][key]
        if not coverage['complete'] or delivery['callbacks'] != len(observed) or delivery['published'] != len(expected):
            raise ValueError('Complete diagnostics and matching original publication counts required')
        stamps = [r['source_ns'] for r in observed]
        if len(stamps) != len(set(stamps)) or set(stamps)-source:
            raise ValueError('Duplicate or unexpected received timestamps')
        if not observed:
            raise ValueError('At least one observed IMU callback required')
        missing = source-set(stamps)
        gaps = [(b['source_ns']-a['source_ns'])/1e6 for a, b in zip(observed, observed[1:])]
        wall_gaps = [(b['begin_monotonic_ns']-a['begin_monotonic_ns'])/1e6
                     for a, b in zip(observed, observed[1:])]
        runs = loss_runs(expected, missing)
        result[name] = dict(expected=len(expected), received=len(observed), missing=len(missing),
            missing_fraction=len(missing)/len(expected),
            missing_before_active=sum(s < start for s in missing),
            missing_in_active=sum(start <= s <= end for s in missing),
            missing_after_active=sum(s > end for s in missing),
            missing_by_source_second=dict(sorted(Counter(s//1_000_000_000 for s in missing).items())),
            longest_missing_run=max((g['count'] for g in runs), default=0),
            loss_runs=runs, source_time_reversals=sum(g < 0 for g in gaps),
            source_gap_ms=distribution(gaps), callback_gap_ms=distribution(wall_gaps),
            source_age_ms=distribution([r['source_age_ms'] for r in observed]),
            callback_duration_ms=distribution([r['duration_ms'] for r in observed]))
        losses[name] = missing
    # 같은 호스트의 다른 수신자가 받았다는 근거이며 DDS 내부 손실 위치는 미관측이다.
    common = losses['local_pursuit'] & losses['lidar_safety']
    result['comparison'] = dict(common_missing=len(common),
        local_missing_but_safety_received=len(losses['local_pursuit']-common),
        safety_missing_but_local_received=len(losses['lidar_safety']-common))
    return result


def publication_evidence(publications, rows, expected):
    stamps = [p['source_ns'] for p in publications]
    if stamps != expected:
        raise ValueError('IMU publication trace must match every original timestamp in order')
    if any(b['begin_monotonic_ns'] < a['end_monotonic_ns'] for a,b in zip(publications,publications[1:])):
        raise ValueError('Invalid serial publication timestamps')
    by_source = {p['source_ns']:p for p in publications}
    intervals = [(b['begin_monotonic_ns']-a['begin_monotonic_ns'])/1e6 for a,b in zip(publications,publications[1:])]
    # 1 ms는 진단용 뭉침 기준이며 센서/제어 허용 기준을 변경하지 않는다.
    clusters, current = [], [publications[0]] if publications else []
    for previous, row in zip(publications, publications[1:]):
        if row['begin_monotonic_ns']-previous['begin_monotonic_ns'] < 1_000_000:
            current.append(row)
        else:
            clusters.append(current)
            current = [row]
    if current: clusters.append(current)
    result = dict(publication_interval_ms=distribution(intervals),
        publish_call_ms=distribution([(p['end_monotonic_ns']-p['begin_monotonic_ns'])/1e6 for p in publications]),
        schedule_lateness_ms=distribution([p['schedule_lateness_wall_ms'] for p in publications]),
        max_submillisecond_cluster=max((len(c) for c in clusters), default=0),
        clusters_at_least_6=sum(len(c)>=6 for c in clusters), nodes={})
    safety = {r['source_ns']:r for r in rows if r['node']=='lidar_safety' and r['callback']=='imu'}
    local_work = sorted((r for r in rows if r['node']=='local_pursuit' and r['callback']!='imu'),
                        key=lambda r:r['begin_monotonic_ns'])
    begins = [r['begin_monotonic_ns'] for r in local_work]
    publication_begins = [p['begin_monotonic_ns'] for p in publications]
    publication_ends = [p['end_monotonic_ns'] for p in publications]
    source = set(expected)
    for name in ('local_pursuit','lidar_safety'):
        observations = [r for r in rows if r['node']==name and r['callback']=='imu']
        missing = source-{r['source_ns'] for r in observations}
        data = dict(publish_end_to_callback_ms=distribution([
            (r['begin_monotonic_ns']-by_source[r['source_ns']]['end_monotonic_ns'])/1e6 for r in observations]),
            missing_in_clusters_at_least_6=sum(p['source_ns'] in missing for c in clusters if len(c)>=6 for p in c))
        ordered = sorted(observations, key=lambda r:r['begin_monotonic_ns'])
        windows, lost_in_windows = [], set()
        for before, after in zip(ordered, ordered[1:]):
            lo = bisect_left(publication_begins, before['begin_monotonic_ns'])
            hi = bisect_left(publication_ends, after['begin_monotonic_ns'])
            published_between = publications[lo:hi]
            if len(published_between) > 5:
                lost = {p['source_ns'] for p in published_between} & missing
                lost_in_windows.update(lost)
                windows.append(dict(publications=len(published_between), missing=len(lost),
                    before_source_ns=before['source_ns'],after_source_ns=after['source_ns'],
                    callback_start_gap_ms=(after['begin_monotonic_ns']-before['begin_monotonic_ns'])/1e6))
        data['windows_more_than_5_publications_between_callbacks'] = windows
        data['missing_published_in_those_windows'] = len(lost_in_windows)
        if name == 'local_pursuit':
            overlap = Counter()
            for stamp in missing:
                if stamp not in safety: continue
                when = safety[stamp]['begin_monotonic_ns']
                index = bisect_left(begins, when)-1
                if index >= 0 and when <= local_work[index]['end_monotonic_ns']:
                    overlap[local_work[index]['callback']] += 1
            data['safety_callback_overlaps_local_work'] = dict(overlap)
        result['nodes'][name] = data
    result['caveat'] = ('Submillisecond clusters and safety-callback overlaps are correlations, '
        'not DDS arrival/queue occupancy measurements or causal proof. Negative publish-end gaps are possible.')
    return result


def compare_probe(expected, rows, probe_rows, publications=None):
    """같은 표본의 수신 교집합. 손실 위치나 인과관계는 단정하지 않는다."""
    if len(expected) != len(set(expected)) or expected != sorted(expected):
        raise ValueError('Unique ordered source timestamps required')
    source = set(expected)
    stamps = [r['source_ns'] for r in probe_rows]
    if len(stamps) != len(set(stamps)) or set(stamps)-source:
        raise ValueError('Duplicate or unexpected probe timestamps')
    missing = source-set(stamps)
    runs = loss_runs(expected, missing)
    result = dict(expected=len(expected), received=len(stamps), missing=len(missing),
        longest_missing_run=max((r['count'] for r in runs), default=0), loss_runs=runs,
        source_time_reversals=sum(b < a for a,b in zip(stamps,stamps[1:])),
        source_gap_ms=distribution([(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]),
        callback_gap_ms=distribution([(b['begin_monotonic_ns']-a['begin_monotonic_ns'])/1e6
            for a,b in zip(probe_rows,probe_rows[1:])]), comparisons={})
    for name in ('local_pursuit', 'lidar_safety'):
        lost = source-{r['source_ns'] for r in rows if r['node']==name and r['callback']=='imu'}
        result['comparisons'][name] = dict(common_missing=len(lost & missing),
            controller_missing_probe_received=len(lost-missing),
            probe_missing_controller_received=len(missing-lost))
    if publications is not None:
        if [p['source_ns'] for p in publications] != expected:
            raise ValueError('Full ordered publication trace required')
        by_stamp = {p['source_ns']:p for p in publications}
        result['publish_end_to_callback_ms'] = distribution([
            (r['begin_monotonic_ns']-by_stamp[r['source_ns']]['end_monotonic_ns'])/1e6 for r in probe_rows])
    return result


def analyze(recording, replay):
    expected = []
    with (recording/'receipt.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['topic'] == '/imu/data':
                expected.append(row['source_ns'])
    report = json.loads((replay/'report.json').read_text(encoding='utf-8'))
    with (replay/'timing.jsonl').open(encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream]
    nodes = analyze_rows(expected, rows, report)
    trace = replay/'imu_publication.jsonl'
    publications = [json.loads(line) for line in trace.open(encoding='utf-8')] if trace.exists() else None
    probe = None
    if report.get('imu_probe_enabled'):
        capture, probe_rows = read_probe_result(replay/'imu_probe')
        if report['imu_probe_result'] != capture:
            raise ValueError('Replay/probe summary mismatch')
        probe = compare_probe(expected, rows, probe_rows, publications)
    return dict(run=replay.parent.name, nodes=nodes,
        minimal_probe=probe,
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        publication=publication_evidence(publications, rows, expected) if publications is not None else None,
        operational_passed=report['operational_passed'],
        scope='Complete callback observations versus original unique timestamps. '
              'DDS receive time/sequence and reader queue drops are not observed; '
              'per-node missing values do not identify the internal loss mechanism. '
              'Header time gaps are not callback calculation times or real vehicle accuracy.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('replays', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    results = [analyze(args.recording, replay) for replay in args.replays]
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(results, stream, indent=2)
    print(json.dumps([dict(run=r['run'], nodes={n: {k:v for k,v in d.items()
        if k in ('missing', 'longest_missing_run', 'source_gap_ms', 'callback_gap_ms')}
        for n,d in r['nodes'].items() if n != 'comparison'}, comparison=r['nodes']['comparison'])
        for r in results]))


if __name__ == '__main__':
    main()
