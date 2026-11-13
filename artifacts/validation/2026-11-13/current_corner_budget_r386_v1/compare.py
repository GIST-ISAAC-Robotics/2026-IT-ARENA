"""원 r377 전체와 보존 집합을 비교한다. ROS/장치/발행/새 주행은 없다."""
import argparse
import collections
import copy
import json
import math
from pathlib import Path
import time

from support import A,ROOT,digest,verify,load


def percentile(values,p):
    if not values:return None
    values=sorted(values);index=(len(values)-1)*p;lo=math.floor(index);hi=math.ceil(index)
    return values[lo]+(values[hi]-values[lo])*(index-lo)


def summary(rows,key):
    values=[r[key] for r in rows if key in r]
    return dict(count=len(values),p50_ms=percentile(values,.5),p95_ms=percentile(values,.95),
        max_ms=max(values) if values else None,above_100ms=sum(v>100 for v in values),above_200ms=sum(v>200 for v in values))


def records(compare):
    root=A.parent
    result=compare.load_trace(root/'released_cad_drive_r377_v1/run1/follow_decisions.jsonl','r377')
    assert len(result)==223
    result.extend(compare.load_trace(root/'relative_body_cad_drive_r371_v1/run1/follow_decisions.jsonl','r371_holdout'))
    result.extend(compare.load_trace(root/'released_crawl_drive_r364_v1/run1/follow_decisions.jsonl','r364_holdout'))
    headers=json.loads((root/'current_input_jetson_r296_v1/headers.json').read_bytes())
    for selected in json.loads((root/'current_input_jetson_r296_v1/selected_inputs.json').read_bytes()):
        row=selected['input'];label=selected['source']
        result.append(dict(label='holdout_'+label,header=headers[label],output=row['output'],
            submitted=compare.plan_reference(row['input']),current=row['current_rebase']['input']))
    assert len(result)==795
    return result


def counterexample_records(original):
    cases=[]
    def add(name,mutate):
        row=copy.deepcopy(original);mutate(row);row['label']='counterexample';cases.append((name,row))
    def sector(row,centers,width,value):
        raw=row['current']['scan']['measurements']
        for i,angle in enumerate(raw['angles_rad']):
            if any(abs(math.remainder(angle-center,math.tau))<math.radians(width) for center in centers):
                raw['ranges_m'][i]=value
    add('measured_speed_015_braking_floor',lambda r:r['current'].__setitem__('speed_mps',.15))
    add('measured_speed_above_scope',lambda r:r['current'].__setitem__('speed_mps',.151))
    add('reverse_measured_speed',lambda r:r['current'].__setitem__('speed_mps',-.001))
    add('both_rear_side_blind',lambda r:sector(r,(-2.2,2.2),24,{'nonfinite':'inf'}))
    add('multiple_front_and_side_obstacles',lambda r:sector(r,(0.,math.pi/2,-math.pi/2),18,.12))
    add('two_front_obstacles',lambda r:sector(r,(-.2,.2),8,.16))
    add('left_near_return',lambda r:sector(r,(math.pi/2,),20,.12))
    add('right_near_return',lambda r:sector(r,(-math.pi/2,),20,.12))
    add('rear_side_nan_not_free',lambda r:sector(r,(-2.2,2.2),24,{'nonfinite':'nan'}))
    add('expired_path_source',lambda r:r['output'][2]['normal'].__setitem__('path_source_s',r['current']['reference_s']-.501))
    add('epoch_changed',lambda r:r['submitted'].__setitem__('epoch',999))
    add('current_time_unverified',lambda r:r['current'].__setitem__('timing_verified',False))
    add('previous_steering_outside_cad',lambda r:r['current'].__setitem__('previous_steering_rad',.46))
    add('dt_outside_contract',lambda r:r['current'].__setitem__('dt_s',.101))
    def both_opposite(r):
        r['current']['previous_steering_rad']=-r['current']['previous_steering_rad']
        sector(r,(-2.2,2.2),24,{'nonfinite':'inf'})
    add('both_blind_opposite_previous_steering',both_opposite)
    return cases


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--backend',choices=('native_v1','python'),default='native_v1')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    assert output.is_relative_to(A) and not output.exists(),'fresh r386 output required'
    output.mkdir();manifest=verify();compare,imports=load(args.backend)
    import candidate
    rows=[];negatives=[];violations=[];error=None;inputs=records(compare)
    def evaluate(record,index,phase):
        original=copy.deepcopy(record)
        cfg=compare.FollowConfig(**record['header']['follow_config'])
        motion=compare.MotionConfig(**record['header']['motion_config'])
        current=record['current'];info,scan,history=compare.restore_call(current,motion)
        scene=compare.motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        call=(record['output'],record['submitted'],current,scene,history,cfg)
        start=time.perf_counter_ns();before=compare.policy.rebase(*call,require_current_region=True)
        baseline_ms=(time.perf_counter_ns()-start)/1e6
        costs={};start=time.perf_counter_ns()
        after=candidate.try_from_current_result(before,*call,core_rebase=compare.policy.core_policy.rebase,
            require_current_region=True,costs=costs)
        added_ms=(time.perf_counter_ns()-start)/1e6
        before,after=compare.freeze(before),compare.freeze(after)
        assert compare.equal_output(record,original),'input mutation'
        diagnostic=after.get(candidate.DIAGNOSTIC) or {};applied=diagnostic.get('applied') is True
        row=dict(phase=phase,index=index,label=record['label'],reference_s=current['reference_s'],
            mode=record['output'][2]['normal'].get('selector_mode'),choice=record['output'][2]['normal'].get('selector_choice'),
            original_target_mps=record['output'][0],measured_speed_mps=current['speed_mps'],
            applied=applied,before=before,after=after,diagnostic=diagnostic,
            baseline_wall_ms=baseline_ms,added_wall_ms=added_ms,comparison_total_wall_ms=baseline_ms+added_ms,**costs)
        rows.append(row)
        if applied:
            assert row['mode']=='SOLO' and row['choice']=='baseline' and before['speed_mps']==0
            assert 0<after['speed_mps']<=.10 and not diagnostic['unknown_filled']
            assert diagnostic['additional_target_attempts']==1 and diagnostic['final_full_checks']==1
            assert all(before.get(k)==after.get(k) for k in ('steering_rad','reference_s','path_source_s','original_reference_s','path_points'))
            assert after['current_route_proof']['accepted']
            assert diagnostic['region']['distance_m']<=diagnostic['proposal']['allowed_distance_m']
            if diagnostic['region']['unobserved']:assert diagnostic['proof']['accepted']
        else:
            assert compare.equal_output(before,{k:v for k,v in after.items() if k!=candidate.DIAGNOSTIC}),'nonapplied changed'
        if phase=='stored' and record['label']=='r377':
            assert compare.equal_output(compare.recorded_result(before,record['expected']),record['expected']),(index,'r377 original reproduction failed')
        elif phase=='stored' and not compare.equal_output(before,{k:v for k,v in after.items() if k!=candidate.DIAGNOSTIC}):
            violations.append(dict(index=index,label=record['label'],reason='preservation_set_changed'))
        return row
    try:
        for index,record in enumerate(inputs):evaluate(record,index,'stored')
        for index,(name,record) in enumerate(counterexample_records(inputs[222])):
            row=evaluate(record,index,'counterexample');row['name']=name
            passed=not row['applied'] and row['after']['speed_mps']==0
            negatives.append(dict(name=name,passed=passed,applied=row['applied'],reason=row['after']['reason'],
                baseline_speed_mps=row['before']['speed_mps'],candidate_speed_mps=row['after']['speed_mps'],
                candidate_reason=row['diagnostic'].get('reason')))
            if not passed:violations.append(dict(name=name,reason='counterexample_not_stopped_or_applied'))
        verify()
    except Exception as exc:
        error=repr(exc);raise
    finally:
        with (output/'rows.jsonl').open('x',encoding='utf8') as stream:
            for row in rows:stream.write(json.dumps(compare.freeze(row),ensure_ascii=False)+'\n')
        primary=[r for r in rows if r['phase']=='stored' and r['label']=='r377']
        applied=[r for r in primary if r['applied']];attempted=[r for r in primary if r['diagnostic']]
        stages=('baseline_wall_ms','added_wall_ms','comparison_total_wall_ms','proposal_geometry_ms','proposal_classification_ms','final_core_ms','final_region_ms','final_gap_ms')
        groups={name:{stage:summary(group,stage) for stage in stages} for name,group in (
            ('r377_all',primary),('r377_attempted',attempted),('r377_applied',applied),('preservation',[r for r in rows if r['phase']=='stored' and r['label']!='r377']))}
        report=dict(passed=error is None and not violations,error=error,violations=violations,
            completed_stored=sum(r['phase']=='stored' for r in rows),completed_counterexamples=len(negatives),
            original_r377_reproduced=len(primary) if error is None else None,
            r377_count=len(primary),r377_attempted=len(attempted),r377_applied=len(applied),
            last=None if len(primary)!=223 else primary[-1],counterexamples=negatives,groups=groups,
            rejection_reasons=dict(collections.Counter(r['diagnostic'].get('reason','outside_scope') for r in primary if not r['applied'])),
            imports=imports,source_count=len(manifest['files']),manifest_sha256=digest(A/'source_manifest.json'),
            candidate_sha256=digest(A/'candidate.py'),budget_sha256=digest(A/'budget.py'),
            automatic_adoption_allowed=False,positive_speed_progress_proven_in_simulation=False,
            full_lap=False,command_expiry_evaluated=False,deadline_permission=False,
            original_run_passed=False,unknown_filled=False,source_timestamps_renewed=False,
            scope='저장 입력 단일 제안/최종검사 비교. 반복 wrapper 운영 연결/새 주행/실차/연속 swept 안전 인증 아님.')
        (output/'report.json').write_text(json.dumps(compare.freeze(report),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        print(json.dumps({k:report[k] for k in ('passed','error','completed_stored','r377_attempted','r377_applied','completed_counterexamples','violations')},ensure_ascii=False))
    if violations:raise SystemExit(1)


if __name__=='__main__':main()
