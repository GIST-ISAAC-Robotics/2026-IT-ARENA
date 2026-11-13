"""legacy795 재현과 finalCAD223 단일후보 비교를 분리한다. ROS/장치/발행 없음."""
import argparse
import collections
import copy
from dataclasses import asdict,replace
import json
from pathlib import Path
import time

from support import A,ROOT,digest,verify,load,original_helpers


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=('frozen','legacy','final'),required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--reference',type=Path)
    args=parser.parse_args();output=args.output.resolve();assert output.is_relative_to(A) and not output.exists()
    output.mkdir();manifest=verify();compare,imports=load('native_v1');helper=original_helpers()
    import candidate
    records=helper.records(compare);assert len(records)==795
    reference=None
    if args.mode!='frozen':
        assert args.reference is not None
        reference=[json.loads(line) for line in (args.reference/'rows.jsonl').open(encoding='utf8')]
        assert len(reference)==795
    selection=None
    if args.mode=='final':
        import yaml
        from arena_autonomy.vehicle_geometry import planning_parameters,preview_parameters
        path=ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
        vehicle=yaml.safe_load(path.read_bytes())['vehicle']
        selection=dict(path=str(path),sha256=digest(path),planning=planning_parameters(vehicle),sender=preview_parameters(vehicle))
    rows=[];violations=[];error=None
    stages=('baseline_wall_ms','added_wall_ms','comparison_total_wall_ms','proposal_geometry_ms',
        'proposal_classification_ms','final_core_ms','final_region_ms','final_gap_ms')
    def require(condition,index,reason):
        if not condition:violations.append(dict(index=index,reason=reason))
    with (output/'rows.jsonl').open('x',encoding='utf8') as stream:
        def evaluate(record,index,phase,name=None):
            original=copy.deepcopy(record)
            cfg=compare.FollowConfig(**record['header']['follow_config']);legacy_cfg=asdict(cfg)
            if selection is not None:
                params=selection['planning'];sender=selection['sender']
                assert (cfg.length_m,cfg.width_m,cfg.wheelbase_m,cfg.max_steering_rad)==(
                    params['vehicle_length_m'],params['vehicle_width_m'],sender['wheelbase_m'],sender['max_steering_angle_rad'])
                cfg=replace(cfg,track_m=params['vehicle_track_width_m'],wheel_radius_m=params['vehicle_wheel_radius_m'],
                    wheel_width_m=params['vehicle_wheel_width_m'])
            config_changes={key:dict(before=value,after=asdict(cfg)[key]) for key,value in legacy_cfg.items() if value!=asdict(cfg)[key]}
            assert set(config_changes)<={'track_m','wheel_radius_m','wheel_width_m'}
            current=record['current'];motion=compare.MotionConfig(**record['header']['motion_config'])
            info,scan,history=compare.restore_call(current,motion)
            scene=compare.motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
                epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
            call=(record['output'],record['submitted'],current,scene,history,cfg)
            start=time.perf_counter_ns();before_raw=compare.policy.rebase(*call,require_current_region=True)
            baseline_ms=(time.perf_counter_ns()-start)/1e6
            before=compare.freeze(before_raw);costs={};added_ms=None
            evaluate_candidate=args.mode=='final' or (args.mode=='legacy' and record['label']!='r377')
            after=before
            if evaluate_candidate:
                start=time.perf_counter_ns()
                after=compare.freeze(candidate.try_from_current_result(before_raw,*call,core_rebase=compare.policy.core_policy.rebase,
                    require_current_region=True,costs=costs))
                added_ms=(time.perf_counter_ns()-start)/1e6
            assert record==original,'saved input mutation'
            diagnostic=after.get(candidate.DIAGNOSTIC) or {};applied=diagnostic.get('applied') is True
            row=dict(index=index,phase=phase,name=name,label=record['label'],reference_s=current['reference_s'],
                mode=record['output'][2]['normal'].get('selector_mode'),choice=record['output'][2]['normal'].get('selector_choice'),
                measured_speed_mps=current['speed_mps'],original_target_mps=record['output'][0],
                before=before,after=after,applied=applied,diagnostic=diagnostic,candidate_evaluated=evaluate_candidate,
                config_changes=config_changes,actual_config=asdict(cfg),input_unchanged=True,baseline_wall_ms=baseline_ms,**costs)
            if added_ms is not None:row.update(added_wall_ms=added_ms,comparison_total_wall_ms=baseline_ms+added_ms)
            if applied:
                require(row['mode']=='SOLO' and row['choice']=='baseline' and before['speed_mps']==0,index,'applied_scope')
                require(0<after['speed_mps']<=.10 and not diagnostic['unknown_filled'],index,'applied_speed_unknown')
                require(diagnostic['additional_target_attempts']==1 and diagnostic['final_full_checks']==1,index,'single_attempt_contract')
                require(all(before.get(k)==after.get(k) for k in ('steering_rad','reference_s','path_source_s','original_reference_s','path_points')),
                    index,'identity_changed')
                require(after['current_route_proof']['accepted'] and diagnostic['region']['distance_m']<=diagnostic['proposal']['allowed_distance_m'],
                    index,'final_contract')
                if diagnostic['region']['unobserved']:require(diagnostic['proof']['accepted'],index,'unknown_proof')
            elif evaluate_candidate:
                require(before=={k:v for k,v in after.items() if k!=candidate.DIAGNOSTIC},index,'nonapplied_not_exact')
            if phase=='stored' and record['label']=='r377' and args.mode!='final':
                row['original_r377_recorded_exact']=compare.recorded_result(before,record['expected'])==record['expected']
                require(row['original_r377_recorded_exact'],index,'original_r377_reproduction')
            if args.mode=='legacy':
                old=reference[index];assert old['index']==index and old['reference_s']==row['reference_s']
                row['legacy_frozen_exact']=before==old['before'];require(row['legacy_frozen_exact'],index,'legacy_changed')
                if evaluate_candidate:require(not applied and before=={k:v for k,v in after.items() if k!=candidate.DIAGNOSTIC},index,'preservation_set_changed')
            if args.mode=='final' and phase=='stored':
                old=reference[index];assert old['index']==index and old['reference_s']==row['reference_s']
                row['legacy_baseline']=old['before'];row['geometry_baseline_changed']=before!=old['before']
                row['geometry_only_positive_recovery']=old['before']['speed_mps']==0 and before['speed_mps']>0
            if phase=='counterexample':
                row['counterexample_passed']=not applied and after['speed_mps']==0
                require(row['counterexample_passed'],index,'counterexample_not_stopped_or_applied:'+name)
            rows.append(row);stream.write(json.dumps(compare.freeze(row),ensure_ascii=False)+'\n');stream.flush()
            return row
        try:
            selected=records[:223] if args.mode=='final' else records
            for index,record in enumerate(selected):evaluate(record,index,'stored')
            if args.mode=='final':
                for index,(name,record) in enumerate(helper.counterexample_records(records[222])):evaluate(record,index,'counterexample',name)
            verify()
        except Exception as exc:error=repr(exc)
    stored=[r for r in rows if r['phase']=='stored'];primary=[r for r in stored if r['label']=='r377']
    attempted=[r for r in primary if r['diagnostic']];applied=[r for r in primary if r['applied']]
    preservation=[r for r in stored if r['label']!='r377'];negatives=[r for r in rows if r['phase']=='counterexample']
    groups={name:{stage:helper.summary(group,stage) for stage in stages} for name,group in (
        ('r377_all',primary),('r377_attempted',attempted),('r377_applied',applied),('preservation',preservation),('counterexamples',negatives))}
    report=dict(schema='r386_final_geometry_separated_comparison',mode=args.mode,passed=error is None and not violations,
        error=error,violations=violations,completed_stored=len(stored),completed_counterexamples=len(negatives),
        original_r377_recorded_exact=sum(r.get('original_r377_recorded_exact') is True for r in primary),
        legacy_frozen_exact=sum(r.get('legacy_frozen_exact') is True for r in stored),preservation_count=len(preservation),
        preservation_applied=sum(r['applied'] for r in preservation),r377_attempted=len(attempted),r377_applied=len(applied),
        geometry_baseline_changed=sum(r.get('geometry_baseline_changed') is True for r in primary),
        geometry_only_positive_recovery=sum(r.get('geometry_only_positive_recovery') is True for r in primary),
        baseline_positive=sum(r['before']['speed_mps']>0 for r in primary),candidate_positive=sum(r['after']['speed_mps']>0 for r in primary),
        counterexamples=[{key:r[key] for key in ('name','counterexample_passed','applied','before','after')} for r in negatives],
        last=primary[-1] if len(primary)==223 else None,groups=groups,selected_geometry=selection,imports=imports,
        rejection_reasons=dict(collections.Counter(r['diagnostic'].get('reason','outside_scope') for r in primary if not r['applied'])),
        candidate_sha256=digest(A/'candidate.py'),budget_sha256=digest(A/'budget.py'),source_manifest_sha256=digest(A/'source_manifest.json'),
        source_count=len(manifest['files']),original_run_passed=False,original_r386_archive_executed=False,
        algorithm_modified=False,automatic_adoption_allowed=False,command_expiry_evaluated=False,deadline_permission=False,
        full_lap=False,new_simulation=False,scope='저장 입력/계산 반례. 원795 legacy 대조와 최종기하223 후보 비교를 분리한다.')
    (output/'report.json').write_text(json.dumps(compare.freeze(report),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:report[k] for k in ('mode','passed','error','completed_stored','completed_counterexamples','r377_attempted','r377_applied','violations')},ensure_ascii=False))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
