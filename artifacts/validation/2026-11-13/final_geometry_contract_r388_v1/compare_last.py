"""r377 마지막 입력의 원소스/기하 전달 legacy/명시 finalCAD를 독립 프로세스로 비교한다."""
import argparse
import copy
from dataclasses import asdict,replace
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def difference(left,right,path=''):
    if type(left) is not type(right):return [dict(path=path,before=left,after=right)]
    if isinstance(left,dict):
        result=[]
        for key in sorted(left.keys()|right.keys()):
            if key not in left or key not in right:result.append(dict(path=path+'/'+key,before=left.get(key),after=right.get(key)))
            else:result.extend(difference(left[key],right[key],path+'/'+key))
        return result
    if isinstance(left,list):
        if len(left)!=len(right):return [dict(path=path,before=left,after=right)]
        return [d for i,(x,y) in enumerate(zip(left,right)) for d in difference(x,y,path+'/'+str(i))]
    return [] if left==right else [dict(path=path,before=left,after=right)]


def load(root):
    for part in reversed(('src/arena_autonomy','src/arena_vehicle_interface')):sys.path.insert(0,str(root/part))
    path=root/'artifacts/validation/2026-11-13/released_current_r375_v1/compare.py'
    spec=importlib.util.spec_from_file_location('r388_frozen_compare',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.configure('native_v1')
    for name,item in list(sys.modules.items()):
        if name.startswith(('arena_autonomy','arena_vehicle_interface','candidate_recheck')) and getattr(item,'__file__',None):
            assert Path(item.__file__).resolve().is_relative_to(root),('external module',name,item.__file__)
    return module


def bounds(points,width,length):
    if not len(points):return dict(count=0)
    import numpy as np
    return dict(count=len(points),x_m=[float(points[:,0].min()),float(points[:,0].max())],
        y_m=[float(points[:,1].min()),float(points[:,1].max())],
        side_exterior_m=[float((np.abs(points[:,1])-width/2).min()),float((np.abs(points[:,1])-width/2).max())],
        front=int(np.sum(points[:,0]>length/2)),rear=int(np.sum(points[:,0]<-length/2)),
        left=int(np.sum(points[:,1]>width/2)),right=int(np.sum(points[:,1]<-width/2)))


def worker(args):
    import numpy as np
    root=args.source_root.resolve();dest=args.output.resolve();dest.mkdir(exist_ok=False)
    compare=load(root)
    from arena_autonomy import pass_release_trial as ps
    from arena_autonomy import _numeric_kernels
    header=json.loads((A/'input_header.jsonl').read_bytes())['data']
    event=json.loads((A/'input_last_call.jsonl').read_bytes())['data']['current_rebase'];plan=event['compute']['plan']
    record=dict(header=header,output=plan['output'],submitted=compare.plan_reference(plan['input']),
        current=event['input'],expected=event['output'])
    original=copy.deepcopy(record);cfg=compare.FollowConfig(**header['follow_config']);original_config=asdict(cfg)
    yaml_hash=None
    if args.mode=='explicit':
        import yaml
        from arena_autonomy.vehicle_geometry import planning_parameters,equivalent_steering_limit
        path=root/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
        selected=yaml.safe_load(path.read_bytes())['vehicle'];mapping=planning_parameters(selected);drive=selected['drivetrain']
        assert cfg.length_m==mapping['vehicle_length_m'] and cfg.width_m==mapping['vehicle_width_m']
        assert cfg.wheelbase_m==drive['wheelbase_m'] and cfg.max_steering_rad==equivalent_steering_limit(drive)
        cfg=replace(cfg,track_m=mapping['vehicle_track_width_m'],wheel_radius_m=mapping['vehicle_wheel_radius_m'],
            wheel_width_m=mapping['vehicle_wheel_width_m']);yaml_hash=digest(path)
    info,scan,history=compare.restore_call(record['current'],compare.MotionConfig(**header['motion_config']))
    scene=compare.motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
        epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
    calls=[];original_query=ps.motion_queries
    def capture(scene,speed,target,steering,previous,dt,follow_cfg,lat_cfg):
        distance,q=original_query(scene,speed,target,steering,previous,dt,follow_cfg,lat_cfg)
        calls.append(dict(queries=q.copy(),scene=scene,lat=lat_cfg,follow=follow_cfg,distance_m=distance,
            speed_mps=speed,target_mps=target,steering_rad=steering,previous_steering_rad=previous,dt_s=dt))
        return distance,q
    ps.motion_queries=capture
    try:
        output=compare.freeze(compare.policy.rebase(record['output'],record['submitted'],record['current'],scene,history,cfg,
            require_current_region=True))
    finally:ps.motion_queries=original_query
    assert record==original,'saved input mutated'
    # 아래 분류/배열 요약은 제어 호출이 끝난 뒤만 수행한다. 원 출력에는 손대지 않는다.
    arrays={};summaries=[]
    for index,call in enumerate(calls):
        q=call['queries'];local_scene=call['scene'];lat=call['lat'];follow=call['follow']
        pad=ps.mx.ray_padding(local_scene,lat,follow.object_motion_bound_mps)
        free=ps.depth_support(local_scene,q,pad)
        masked=np.zeros(len(q),dtype=bool)
        if len(q):masked[~free]=ps.mx.masked_only(local_scene,q[~free],pad)
        unknown=~free & ~masked
        arrays[f'q_{index}']=q;arrays[f'free_{index}']=free;arrays[f'masked_{index}']=masked
        summaries.append(dict(index=index,**{k:v for k,v in call.items() if k not in ('queries','scene','lat','follow')},
            all=bounds(q,follow.width_m,follow.length_m),unknown=bounds(q[unknown],follow.width_m,follow.length_m),
            masked=bounds(q[masked],follow.width_m,follow.length_m),lateral_config=asdict(lat)))
    np.savez_compressed(dest/'queries.npz',**arrays)
    recorded=compare.recorded_result(output,record['expected'])
    imported={}
    for item in list(sys.modules.values()):
        filename=getattr(item,'__file__',None)
        if filename:
            path=Path(filename).resolve()
            if path.is_file() and path.is_relative_to(root) and path.suffix in ('.py','.so'):
                imported[path.relative_to(root).as_posix()]=digest(path)
    report=dict(mode=args.mode,source_root=str(root),record_reference_s=record['current']['reference_s'],
        measured_speed_mps=record['current']['speed_mps'],original_target_mps=record['output'][0],
        config=asdict(cfg),config_changes=difference(original_config,asdict(cfg)),lateral_config=asdict(ps.lateral_config(cfg)),
        yaml_sha256=yaml_hash,output=output,recorded_output=recorded,expected=record['expected'],
        recorded_comparator_equal=compare.equal_output(recorded,record['expected']),
        recorded_exact_equal=recorded==record['expected'],recorded_differences=difference(record['expected'],recorded),
        query_calls=summaries,input_unchanged=record==original,source_hashes=imported,
        native_sha256=digest(Path(_numeric_kernels.__file__)),candidate_chain_unchanged=True,
        r386_included=False,original_run_passed=False,new_simulation=False)
    (dest/'report.json').write_text(json.dumps(compare.freeze(report),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(mode=args.mode,speed_mps=output['speed_mps'],reason=output['reason'],query_calls=len(calls),
        recorded_comparator_equal=report['recorded_comparator_equal'],recorded_exact_equal=report['recorded_exact_equal'])))


def aggregate(args):
    import numpy as np
    output=args.output.resolve();reports={key:json.loads((output/key/'report.json').read_bytes())
        for key in ('frozen','legacy','explicit')}
    old,new=reports['legacy'],reports['explicit'];changes=[]
    with np.load(output/'legacy/queries.npz') as left,np.load(output/'explicit/queries.npz') as right:
        used=set()
        signature=lambda c:tuple(c[k] for k in ('speed_mps','target_mps','steering_rad','previous_steering_rad','dt_s','distance_m'))
        for before in old['query_calls']:
            matches=[c for c in new['query_calls'] if c['index'] not in used and signature(c)==signature(before)]
            if not matches:
                changes.append(dict(before_index=before['index'],after_index=None,reason='call_not_reached_in_new_geometry'));continue
            after=matches[0];used.add(after['index']);i,j=before['index'],after['index']
            lq,rq=left[f'q_{i}'],right[f'q_{j}'];ls={tuple(p) for p in lq};rs={tuple(p) for p in rq}
            only_l=np.asarray(sorted(ls-rs)).reshape(-1,2);only_r=np.asarray(sorted(rs-ls)).reshape(-1,2)
            changed=dict(before_index=i,after_index=j,call_signature=signature(before),common_exact_xy=len(ls&rs),
                removed=bounds(only_l,.15,.207),added=bounds(only_r,.15,.207),before=before,after=after)
            for label,arr,q,other in (('removed',left,lq,rs),('added',right,rq,ls)):
                index=i if label=='removed' else j
                mask=np.asarray([tuple(p) not in other for p in q],dtype=bool)
                free=arr[f'free_{index}'];masked=arr[f'masked_{index}']
                changed[label+'_unknown']=bounds(q[mask & ~free & ~masked],.15,.207)
                changed[label+'_masked']=bounds(q[mask & masked],.15,.207)
            changes.append(changed)
        for row in new['query_calls']:
            if row['index'] not in used:changes.append(dict(before_index=None,after_index=row['index'],reason='new_geometry_extra_call',after=row))
    frozen=reports['frozen'];checks=dict(original_r377_comparator_reproduced=frozen['recorded_comparator_equal'],
        original_r377_exact_reproduced=frozen['recorded_exact_equal'],legacy_staged_output_exact=frozen['output']==old['output'],
        legacy_staged_query_arrays_exact=False,inputs_unchanged=all(r['input_unchanged'] for r in reports.values()),
        same_native_binary=len({r['native_sha256'] for r in reports.values()})==1)
    with np.load(output/'frozen/queries.npz') as left,np.load(output/'legacy/queries.npz') as right:
        checks['legacy_staged_query_arrays_exact']=set(left.files)==set(right.files) and all(np.array_equal(left[k],right[k]) for k in left.files)
    changed_config={d['path'] for d in new['config_changes']}
    checks['explicit_changes_only_selected_dimensions']=changed_config<= {'/track_m','/wheel_radius_m','/wheel_width_m'}
    report=dict(schema='r388_final_saved_input_geometry_comparison',passed=all(checks.values()),checks=checks,
        original_run_passed=False,automatic_adoption=False,r386_included=False,new_simulation=False,
        original_trace_current_count=223,selected_index=222,reference_s=new['record_reference_s'],
        original_expected=frozen['expected'],frozen_output=frozen['output'],legacy_output=old['output'],explicit_output=new['output'],
        original_reproduction_differences=frozen['recorded_differences'],
        legacy_differences=difference(frozen['output'],old['output']),geometry_output_differences=difference(old['output'],new['output']),
        geometry_config_changes=new['config_changes'],query_range_changes=changes,
        limitations='마지막 저장1입력의 현재 검사만. 미래 계획/센서/이력의 재생성, 개선 주행, 전체랩 또는 실물 증명 아님.')
    (output/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(passed=report['passed'],checks=checks,before_speed=old['output']['speed_mps'],after_speed=new['output']['speed_mps'])))
    if not report['passed']:raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=('frozen','legacy','explicit','aggregate'),required=True)
    parser.add_argument('--source-root',type=Path);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.mode=='aggregate':aggregate(args)
    else:worker(args)


if __name__=='__main__':main()
