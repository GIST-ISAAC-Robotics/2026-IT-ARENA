#!/usr/bin/env python3
"""저장 G1 입력의 관측 조각/출처 보존 감사. 분류 정확도나 주행 시험이 아니다."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import zipfile

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.observed_fragments import FragmentConfig, partition_observed
from arena_autonomy.scene_motion import motion_scene

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def audit(events):
    count, calls, reused = 0, 0, 0
    motion = footer = None
    seen, rows, errors = {}, [], []
    for event in events:
        if footer is not None or event.get('seq') != count:
            raise ValueError('trace gap, reorder, or trailing data')
        count += 1
        kind, data = event['kind'], event['data']
        if count == 1:
            if kind != 'header' or data.get('schema') != 1 or data.get('scope') != 'g1_decision_stage_only':
                raise ValueError('unsupported trace header')
            motion = MotionConfig(**data['motion_config'])
        elif kind == 'reset':
            # 전후 같은 시각의 광선을 새 관측으로 세지 않는다. epoch가 달라지면 별개다.
            continue
        elif kind == 'call':
            calls += 1
            inputs = data['input']
            scan = inputs['scan']
            key = (inputs['epoch'], scan['frame_id'], scan['sec'], scan['nanosec'])
            fingerprint = hashlib.sha256(json.dumps(scan, sort_keys=True, allow_nan=False).encode()).hexdigest()
            if key in seen:
                if fingerprint != seen[key]:
                    raise ValueError('same source key has different raw scan')
                reused += 1
                continue
            seen[key] = fingerprint
            values, restored, history = restore_call(inputs, motion)
            try:
                scene = motion_scene(restored, history, values['reference_s'],
                    timing_verified=values['timing_verified'], epoch=values['epoch'],
                    rear_half_angle_deg=values['rear_half_angle_deg'])
            except MotionUnavailable as error:
                errors.append(dict(seq=event['seq'], source_key=key, reason=str(error)))
                continue
            partition = partition_observed(scene)
            returned = tuple(ray for ray in scene.rays if ray.status == 'return')
            partitioned = tuple(ray for fragment in partition.fragments for ray in fragment.rays)
            if partitioned != returned:
                raise AssertionError('return identity, endpoint, acquisition or order changed')
            if partition.unknown_indices != tuple(r.source_index for r in scene.rays if r.status == 'unknown'):
                raise AssertionError('unknown evidence changed')
            if partition.masked_indices != tuple(r.source_index for r in scene.rays if r.status == 'masked'):
                raise AssertionError('mask evidence changed')
            rows.append(dict(seq=event['seq'], reference_s=scene.t_ref_s,
                source_first_s=scene.source_first_s, source_last_s=scene.source_last_s,
                source_epoch=scene.source_epoch, returns=len(returned),
                unknown=len(partition.unknown_indices), masked=len(partition.masked_indices),
                fragments=len(partition.fragments), singletons=sum(len(f.rays)==1 for f in partition.fragments),
                stale_fragments=sum(not f.fresh for f in partition.fragments),
                maximum_fragment_span_s=max((f.source_last_s-f.source_first_s for f in partition.fragments), default=0.),
                maximum_fragment_points=max((len(f.rays) for f in partition.fragments), default=0),
                conservation_passed=True))
        elif kind == 'footer':
            footer = data
        else:
            raise ValueError('unknown trace event')
    if footer is None or calls == 0:
        raise ValueError('missing footer or no calls')
    if (any(footer.get(k) != 0 for k in ('capture_errors','failed','dropped','pending'))
            or footer.get('worker_alive') is not False
            or footer.get('written') != count-1 or footer.get('accepted') != count-1):
        raise ValueError('incomplete trace capture')
    totals = {key: sum(row[key] for row in rows)
              for key in ('returns','unknown','masked','fragments','singletons','stale_fragments')}
    return dict(scope='source_partition_contract_not_classification_or_tracking', calls=calls,
        unique_scans=len(seen), reused_calls=reused, transformed_scans=len(rows), rows=rows,
        projection_errors=errors, projection_error_counts=dict(Counter(e['reason'] for e in errors)),
        capture_integrity=True, partition_conservation_passed=bool(rows),
        all_unique_scans_projected=not errors, totals=totals,
        driving_authorized=False, free_area_certified=False,
        config=asdict(FragmentConfig()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    names = ['scripts/audit_observed_fragments.py', 'tests/test_observed_fragments.py',
        'tests/test_observed_fragment_audit.py',
        *['src/arena_autonomy/arena_autonomy/'+name+'.py' for name in
          ('observed_fragments','local_scene','scene_motion','lidar_motion','lidar_observation','follow_trace')]]
    sources = {name:(ROOT/name).read_bytes() for name in names}
    with zipfile.ZipFile(output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for name,data in sources.items(): bundle.writestr(name,data)
    results=[]
    for i,path in enumerate(args.trace):
        before=digest(path)
        with path.open(encoding='utf-8') as stream:
            result=audit(json.loads(line) for line in stream)
        result.update(input_sha256=before,input_unchanged=digest(path)==before,
                      input_path=str(path))
        (output/f'trace_{i}.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
        results.append({k:v for k,v in result.items() if k!='rows'})
    unchanged=all((ROOT/name).read_bytes()==data for name,data in sources.items())
    report=dict(results=results, sources_unchanged=unchanged,
                source_sha256={name:hashlib.sha256(data).hexdigest() for name,data in sources.items()},
                driving_authorized=False)
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(sources_unchanged=unchanged, traces=[dict(unique=r['unique_scans'],
        projected=r['transformed_scans'], totals=r['totals'], errors=r['projection_error_counts']) for r in results])))
    return 0 if unchanged and all(r['input_unchanged'] and r['partition_conservation_passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
