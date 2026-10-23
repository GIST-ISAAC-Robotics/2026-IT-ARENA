#!/usr/bin/env python3
"""현재 기준 판단+회랑 관측기의 단일 PC 비용 비교. ROS/Jetson 마감 검증 아님."""
import argparse
import gc
import json
from pathlib import Path
import time
import zipfile

from arena_autonomy.corridor_observer import CorridorObserver
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from probe_corner_corridor import ROOT,digest
from profile_follow_trace import GCObservation, distribution


def timed_pair(follow,observer,raw,motion,clock=time.perf_counter_ns,gc_observer=None):
    cpu_begin=time.thread_time_ns()
    gc_marks=[gc_observer.elapsed_ns] if gc_observer else None
    gc_count=gc_observer.collections if gc_observer else 0
    begin=clock()
    data,scan,history=restore_call(raw,motion)
    restored=clock()
    if gc_marks is not None:gc_marks.append(gc_observer.elapsed_ns)
    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                       epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    projected=clock()
    if gc_marks is not None:gc_marks.append(gc_observer.elapsed_ns)
    args=(scene,history,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'])
    baseline=follow.command(*args)
    baseline_done=clock()
    if gc_marks is not None:gc_marks.append(gc_observer.elapsed_ns)
    proposal=observer.evaluate(*args)
    end=clock()
    cost=dict(restore_ms=(restored-begin)/1e6,projection_ms=(projected-restored)/1e6,
        baseline_ms=(baseline_done-projected)/1e6,observer_ms=(end-baseline_done)/1e6,
        decision_pair_ms=(end-projected)/1e6,total_ms=(end-begin)/1e6)
    if gc_marks is not None:
        gc_marks.append(gc_observer.elapsed_ns)
        cost.update(thread_cpu_ms=(time.thread_time_ns()-cpu_begin)/1e6,
                    gc_ms=(gc_marks[-1]-gc_marks[0])/1e6,gc_collections=gc_observer.collections-gc_count,
                    gc_by_stage_ms={name:(b-a)/1e6 for name,a,b in
                        zip(('restore','projection','baseline','observer'),gc_marks,gc_marks[1:])})
    return baseline,proposal,cost


def profile(events,expected):
    observation=GCObservation()
    callback=observation.callback
    gc.callbacks.append(callback)
    try:
        return _profile(events,expected,observation)
    finally:
        gc.callbacks.remove(callback)


def _profile(events,expected,observation):
    old={r['seq']:r for r in expected['rows']}
    follow=observer=motion=None
    rows=[]
    for event in events:
        data=event['data']
        if event['kind']=='header':
            config=FollowConfig(**data['follow_config'])
            follow,observer=LocalFollow(config),CorridorObserver(config)
            motion=MotionConfig(**data['motion_config'])
        elif event['kind']=='reset':
            follow.reset(); observer.reset()
        elif event['kind']=='call':
            # 불가 입력을 0시간 성공으로 세지 않는다. 예외가 생기면 표본 보존 도구 실패로 중단.
            baseline,proposal,cost=timed_pair(follow,observer,data['input'],motion,gc_observer=observation)
            prior=old[event['seq']]
            rows.append(dict(seq=event['seq'],time_s=data['input']['reference_s'],
                baseline_matches=equal_output(freeze(baseline),data['output']),
                observer_matches=equal_output(freeze(proposal),prior['proposal']),
                fresh=proposal['fresh'],positive=proposal['positive'],cost=cost))
    if {r['seq'] for r in rows}!=set(old) or len(rows)!=len(old):
        raise ValueError('saved call set mismatch')
    keys=('restore_ms','projection_ms','baseline_ms','observer_ms','decision_pair_ms','total_ms','thread_cpu_ms','gc_ms')
    summaries={label:{k:distribution([r['cost'][k] for r in selected]) for k in keys} if selected else None
               for label,selected in (('all',rows),('fresh',[r for r in rows if r['fresh']]),
                                       ('positive',[r for r in rows if r['positive']]))}
    return dict(calls=len(rows),baseline_matched=sum(r['baseline_matches'] for r in rows),
                observer_matched=sum(r['observer_matches'] for r in rows),cost_ms=summaries,rows=rows,
                gc_collections_during_timed_calls=sum(r['cost']['gc_collections'] for r in rows),
                gc_policy_changed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--observer-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    expected=json.loads(args.observer_report.read_text())
    inputs={str(p):digest(p) for p in (args.input,args.observer_report)}
    if digest(args.input) not in expected['input_sha256'].values() or not expected['source_unchanged']:
        raise ValueError('observer provenance mismatch')
    sources={**expected['source_sha256'],**{p:digest(ROOT/p) for p in
        ('scripts/profile_corridor_observer.py','scripts/profile_follow_trace.py','tests/test_corridor_profile.py')}}
    for p,h in sources.items():
        if digest(ROOT/p)!=h:raise ValueError(f'source mismatch: {p}')
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(ROOT/p,p)
    with args.input.open() as stream:result=profile((json.loads(line) for line in stream),expected)
    result.update(scope='one_pc_native_gc_pass_no_warmup',source_sha256=sources,input_sha256=inputs,
        source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
        input_unchanged=all(digest(p)==h for p,h in inputs.items()),runtime_changed=False,
        includes_json_parse=False,includes_output_comparison=False,includes_independent_guard=False,
        jetson_verified=False,ros_deadline_verified=False)
    with (args.output/'report.json').open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','source_sha256','input_sha256')}))
    return 0 if (result['calls']==result['baseline_matched']==result['observer_matched'] and
                 result['source_unchanged'] and result['input_unchanged']) else 2


if __name__=='__main__':raise SystemExit(main())
