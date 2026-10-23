#!/usr/bin/env python3
"""새 관측기와 보존 시간축 shadow 동등성 및 기본 HOLD 대체 범위의 기록 대조."""
import argparse
import json
from pathlib import Path
import zipfile

from arena_autonomy.corridor_observer import CorridorObserver, compare_boundary_hold
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from probe_corner_corridor import ROOT,digest
from probe_corner_temporal import baseline_matches


def inspect_events(events,temporal):
    expected={r['seq']:r for r in temporal['rows']}
    observer=motion=None
    rows=[]
    for event in events:
        data=event['data']
        if event['kind']=='header':
            observer=CorridorObserver(FollowConfig(**data['follow_config']))
            motion=MotionConfig(**data['motion_config'])
        elif event['kind']=='reset':
            observer.reset()
        elif event['kind']=='call':
            inputs,scan,history=restore_call(data['input'],motion)
            try:
                scene=motion_scene(scan,history,inputs['reference_s'],timing_verified=inputs['timing_verified'],
                    epoch=inputs['epoch'],rear_half_angle_deg=inputs['rear_half_angle_deg'])
                result=observer.evaluate(scene,history,inputs['speed_mps'],inputs['side'],inputs['previous_steering_rad'],inputs['dt_s'])
            except (ValueError,MotionUnavailable) as error:
                observer.reset()
                result=dict(positive=False,reason='input_rejected',error=str(error),proposed_speed_mps=0.,fresh=False)
            old=expected[event['seq']]['shadow']
            keys=('reason','fresh','confirmations','release_count','proposed_speed_mps')
            matched=result['positive']==old['temporal_candidate'] and all(result.get(k)==old.get(k) for k in keys)
            comparison=compare_boundary_hold(data['output'],result)
            rows.append(dict(seq=event['seq'],time_s=inputs['reference_s'],matched=matched,
                             positive=result['positive'],proposal=result,comparison=comparison))
    if len(rows)!=len(expected):raise ValueError('saved calls missing')
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--stateless',type=Path,required=True)
    parser.add_argument('--temporal',type=Path,required=True)
    parser.add_argument('--module',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    inputs={str(p):digest(p) for p in (args.input,args.stateless,args.temporal,args.module)}
    stateless,temporal,module=[json.loads(p.read_text()) for p in (args.stateless,args.temporal,args.module)]
    baseline_matches(stateless,args.input)
    if temporal['input_sha256']!=digest(args.input) or temporal['baseline_sha256']!=digest(args.stateless):
        raise ValueError('temporal source mismatch')
    sources={**temporal['source_sha256'],**module['source_sha256'],
             **{p:digest(ROOT/p) for p in ('src/arena_autonomy/arena_autonomy/corridor_observer.py',
                'scripts/compare_corridor_observer.py','tests/test_corridor_observer.py')}}
    for p,h in sources.items():
        if digest(ROOT/p)!=h:raise ValueError(f'source mismatch: {p}')
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(ROOT/p,p)
    with args.input.open() as stream:rows=inspect_events((json.loads(line) for line in stream),temporal)
    summary=dict(calls=len(rows),matched=sum(r['matched'] for r in rows),positive=sum(r['positive'] for r in rows),
                 selected_for_boundary_hold=sum(r['comparison']['selected'] for r in rows))
    report=dict(scope='observer_equivalence_and_offline_output_selection',summary=summary,rows=rows,
                input_sha256=inputs,source_sha256=sources,
                input_unchanged=all(digest(p)==h for p,h in inputs.items()),
                source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
                runtime_changed=False,driving_authorized=False,baseline_reexecution=False,
                closed_loop_verified=False,transition_dynamics_verified=False)
    with (args.output/'report.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({**summary,'input_unchanged':report['input_unchanged'],'source_unchanged':report['source_unchanged']}))
    return 0 if summary['matched']==len(rows) and report['input_unchanged'] and report['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
