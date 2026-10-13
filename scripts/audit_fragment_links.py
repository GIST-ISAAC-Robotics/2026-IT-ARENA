#!/usr/bin/env python3
"""저장 입력의 조건부 관측 조각 연관 후보. 실제 물체 identity는 알지 못한다."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import zipfile

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.fragment_links import LinkBounds, candidate_links
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.scene_motion import motion_scene
from audit_observed_fragments import ROOT, audit, digest


def events(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def analyze(path, bounds):
    # 기존 감사의 기록 무결성 조건을 재사용한다. 기록 통과와 연관 품질을 섞지 않는다.
    checked=audit(events(path))
    previous=None
    seen=set()
    rows=[]
    for event in events(path):
        if event['kind']=='header':
            config=MotionConfig(**event['data']['motion_config'])
        elif event['kind']=='reset':
            previous=None
        elif event['kind']=='call':
            values,scan,history=restore_call(event['data']['input'],config)
            key=(values['epoch'],scan.header.frame_id,scan.header.stamp.sec,scan.header.stamp.nanosec)
            if key in seen:
                continue
            seen.add(key)
            try:
                scene=motion_scene(scan,history,values['reference_s'],timing_verified=values['timing_verified'],
                    epoch=values['epoch'],rear_half_angle_deg=values['rear_half_angle_deg'])
            except MotionUnavailable as error:
                rows.append(dict(seq=event['seq'],evaluated=False,reason='projection:'+str(error)))
                previous=None
                continue
            if previous is not None:
                started=time.perf_counter_ns()
                graph=candidate_links(previous,scene,history,bounds)
                elapsed=(time.perf_counter_ns()-started)/1e6
                rows.append(dict(seq=event['seq'],evaluated=graph.evaluated,reason=graph.reason,
                    links=len(graph.links),ambiguous_previous=len(graph.ambiguous_previous),
                    ambiguous_current=len(graph.ambiguous_current),unmatched_previous=len(graph.unmatched_previous),
                    unmatched_current=len(graph.unmatched_current),previous_source_first_s=graph.previous_source_first_s,
                    current_source_first_s=graph.current_source_first_s,elapsed_ms=elapsed))
            previous=scene
    evaluated=[row for row in rows if row['evaluated']]
    return dict(scope='stored_sensor_input_compatibility_graph_not_tracking_accuracy',
        input_capture_integrity=checked['capture_integrity'],unique_scans=len(seen),rows=rows,
        pair_attempts=len(rows),evaluated_pairs=len(evaluated),
        reasons=dict(Counter(row['reason'] for row in rows)),
        pairs_with_ambiguous_links=sum(bool(row['ambiguous_previous'] or row['ambiguous_current']) for row in evaluated),
        maximum_links=max((row['links'] for row in evaluated),default=None),
        maximum_elapsed_ms=max((row['elapsed_ms'] for row in evaluated),default=None),
        mean_elapsed_ms=sum(row['elapsed_ms'] for row in evaluated)/len(evaluated) if evaluated else None,
        config=asdict(bounds),identity_certified=False,stationary_certified=False,
        free_area_certified=False,driving_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    output.mkdir(parents=True,exist_ok=False)
    names=['scripts/audit_fragment_links.py','scripts/audit_observed_fragments.py','tests/test_fragment_links.py',
        *['src/arena_autonomy/arena_autonomy/'+name+'.py' for name in
        ('fragment_links','observed_fragments','local_scene','scene_motion','lidar_motion','lidar_observation','follow_trace')]]
    sources={name:(ROOT/name).read_bytes() for name in names}
    with zipfile.ZipFile(output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for name,value in sources.items(): bundle.writestr(name,value)
    # 임시 탐색 가정. 성능을 좋게 보이려고 실제 표적 label/속도를 공급하지 않는다.
    bounds=LinkBounds(object_diameter_m=.25,speed_bound_mps=1.,range_error_m=.03,ego_transport_error_m=.03)
    results=[]
    for i,path in enumerate(args.trace):
        before=digest(path)
        result=analyze(path,bounds)
        result.update(input_path=str(path),input_sha256=before,input_unchanged=before==digest(path))
        (output/f'trace_{i}.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
        results.append({k:v for k,v in result.items() if k!='rows'})
    report=dict(results=results,source_sha256={name:hashlib.sha256(value).hexdigest() for name,value in sources.items()},
        sources_unchanged=all((ROOT/name).read_bytes()==value for name,value in sources.items()),driving_authorized=False,
        limits=['Graph candidates use measured ego history, not target labels or map/world pose.',
            'A witness only permits a possible association under supplied size/motion/error assumptions.',
            'All ambiguous edges are kept; a unique edge is not certified identity or zero velocity.',
            'Unmatched/expired support is not evidence of empty space.',
            'Costs are offline PC calls, not ROS deadlines or Jetson capacity.'])
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(report,indent=2,allow_nan=False))
    return 0 if report['sources_unchanged'] and all(r['input_unchanged'] for r in results) else 1


if __name__=='__main__':
    raise SystemExit(main())
