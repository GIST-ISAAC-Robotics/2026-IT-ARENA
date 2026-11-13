"""동결 r469 plan 원순서와 실제 current를 읽기 전용 재생한다.

후보 함수는 원래대로 호출하고 결과를 바꾸지 않는다. 진단은 이 프로세스의
wrapper와 새 artifact에만 남는다. ROS 초기화/발행/새 주행은 없다.
"""
from pathlib import Path
import collections
import hashlib
import json
import math
import sys

A = Path(__file__).resolve().parent
S = A / 'stage'
sys.path[:0] = [str(S/'src/arena_autonomy'), str(S/'src/arena_vehicle_interface')]
import numpy as np
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy import pass_release_trial as ps, released_body_gap as body
from arena_autonomy.follow_trace import freeze
from arena_autonomy.rear_fringe_trial import _query_brackets
from arena_autonomy.scan_input import scan_angles
from arena_autonomy.runtime_dual_state import current_request

OUT = A / (sys.argv[1] if len(sys.argv)>1 else 'diagnostic_v1')
OUT.mkdir(exist_ok=False)
TRACE = A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'
trace_before = hashlib.sha256(TRACE.read_bytes()).hexdigest()
manifest = json.loads((A/'source_manifest.json').read_text(encoding='utf-8'))['files']
assert all(hashlib.sha256((S/p).read_bytes()).hexdigest()==sha for p,sha in manifest.items())
rows = [json.loads(line) for line in TRACE.open(encoding='utf-8')]
header = next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header')
submitted = [r['data'] for r in rows if r['kind']=='runtime_plan_submitted']
completed = {(r['data']['generation'],r['data']['sequence']):r['data'] for r in rows if r['kind']=='runtime_plan_completed'}
assert len(submitted)==len(completed)==217
first_current = next(r for r in rows if r['seq']==823)['data']['current_rebase']
stall = [x for x in submitted if x['input']['reference_s']>=19.22]
representatives = {stall[i]['input']['reference_s'] for i in (0,len(stall)//2,len(stall)-1)}

active = {}
events = []
region_records = []
original_region, original_check, original_propose = ps.motion_region, body.check, body.propose


def scalar(value):
    return {k:v for k,v in value.items() if not isinstance(v,(dict,list,tuple,np.ndarray))}


def point_summary(q, cfg):
    if not len(q): return {'count':0}
    ext=np.abs(q[:,1])-cfg.width_m/2
    return dict(count=len(q), x_min_m=float(q[:,0].min()), x_max_m=float(q[:,0].max()),
        y_min_m=float(q[:,1].min()), y_max_m=float(q[:,1].max()),
        exterior_min_m=float(ext.min()), exterior_max_m=float(ext.max()),
        outside_front=int(np.sum(q[:,0]>cfg.length_m/2)),
        outside_rear=int(np.sum(q[:,0]<-cfg.length_m/2)),
        exterior_above_1cm=int(np.sum(ext>.01)))


def geometry(scene, raw, cfg, context):
    if not context: return {'context_present':False}
    q=context['queries']; pad=context['padding']; free=context['free']; masked=context['masked']
    unknown=q[~free & ~masked]
    result=dict(frame='current ego base_link; no world truth',
        source_first_s=scene.source_first_s, reference_s=scene.t_ref_s,
        all_queries=point_summary(q,cfg), unknown_queries=point_summary(unknown,cfg),
        masked_queries=point_summary(q[masked],cfg))
    if not len(unknown):return result
    rays=scene.rays; origins=np.asarray([r.ray_origin_xy_m for r in rays]);directions=np.asarray([r.direction_xy for r in rays])
    status=[r.status for r in rays]; ranges=np.asarray(raw.ranges,dtype=float);angles=np.asarray(scan_angles(raw))
    indices=set(); pairs=collections.Counter(); sample=[]
    for j,(radius,rel,lo,hi) in enumerate(_query_brackets(unknown,origins,directions)):
        pairs[status[lo]+'|'+status[hi]]+=1;indices.update((lo,hi))
        if len(sample)<5:
            sample.append(dict(query_xy_m=unknown[j].tolist(), lo=lo,hi=hi,
                lo_status=status[lo],hi_status=status[hi],
                bracket_gap_deg=math.degrees(float(abs(rel[lo])+abs(rel[hi])))))
    result['bracket_status_counts']=dict(pairs);result['query_examples']=sample
    def raw_kind(v):
        return 'nan' if math.isnan(v) else 'positive_inf' if v==math.inf else 'negative_inf' if v==-math.inf else 'finite'
    groups=[]
    for i in sorted(indices):
        if not groups or i!=groups[-1][-1]+1:groups.append([i])
        else:groups[-1].append(i)
    result['source_bracket_groups']=[dict(first=g[0],last=g[-1],
        source_angles_deg=[math.degrees(float(angles[g[0]])),math.degrees(float(angles[g[-1]]))],
        statuses=dict(collections.Counter(status[i] for i in g)),
        raw_kinds=dict(collections.Counter(raw_kind(float(ranges[i])) for i in g)),
        finite_ranges_m=[float(ranges[i]) for i in g if math.isfinite(ranges[i])],
        transformed_angles_deg=[math.degrees(math.atan2(rays[i].direction_xy[1],rays[i].direction_xy[0])) for i in (g[0],g[-1])]) for g in groups]
    valid=[i for i,r in enumerate(rays) if r.status=='return']
    if valid:
        hits=np.asarray([rays[i].hit_xy_m for i in valid]); margins=np.linalg.norm(unknown[:,None,:]-hits[None,:,:],axis=2)-pad[valid]
        result['minimum_known_return_padding_margin_m']=float(margins.min())
        result['queries_inside_known_return_padding']=int(np.sum(np.any(margins<=0,axis=1)))
    result['raw_unknown_ray_count']=sum(s=='unknown' for s in status)
    result['raw_unknown_reasons']=dict(collections.Counter(raw_kind(float(ranges[i])) for i,s in enumerate(status) if s=='unknown'))
    return result


def region_wrapper(*args, **kwargs):
    result=original_region(*args,**kwargs)
    if active.get('representative') and kwargs.get('context'):
        scene,speed,target,steering,previous,dt,cfg,_=args
        region_records.append(dict(region=result,scene=scene,context=kwargs['context'],cfg=cfg,
            speed=speed,target=target,steering=steering,previous=previous,dt=dt))
    return result


def check_wrapper(*args, **kwargs):
    scene,scan,speed,target,steering,previous,dt,cfg,_,_=args
    result=original_check(*args,**kwargs)
    item=dict(kind='body_check', stage=active.get('stage'), reference_s=active.get('reference_s'),
        speed_mps=speed,target_mps=target,steering_rad=steering,previous_rad=previous,dt_s=dt,
        result=freeze(result))
    if active.get('representative'):
        match=next((r for r in reversed(region_records) if r['scene'] is scene and r['target']==target and r['region']==result.get('region')),None)
        if match:item['geometry']=geometry(scene,scan,cfg,match['context'])
    events.append(item)
    return result


def propose_wrapper(*args, **kwargs):
    start=len(events)
    owner,scene,scan,baseline,speed,previous,dt,limit=args
    result=original_propose(*args,**kwargs)
    checks=[x for x in events[start:] if x['kind']=='body_check']
    events.append(dict(kind='propose',stage=active.get('stage'),reference_s=active.get('reference_s'),
        speed_mps=speed,limit=limit,baseline_command=baseline[:2],mode=owner.mode,
        partial=scalar(baseline[2].get('partial_observation') or {}),
        check_targets=[x['target_mps'] for x in checks],result=freeze(result),
        skipped_targets=[dict(target=t,limit=t>limit,baseline=t>=baseline[0],speed=speed>t,
            guard_cap=t>owner.config.guard_speed_cap_mps) for t in (.35,.25,.15) if all(t!=x['target_mps'] for x in checks)]))
    return result


ps.motion_region=region_wrapper;body.check=check_wrapper;body.propose=propose_wrapper
e=Evaluator('plan',header)
generation=None; comparisons=[]; representative_geometry=[]
try:
    for item in submitted:
        key=(item['generation'],item['sequence']);data=item['input'];ref=data['reference_s']
        if generation!=key[0]:e.reset(item['reset_reason']);generation=key[0]
        active.update(stage='plan',reference_s=ref,representative=ref in representatives)
        region_records.clear()
        before=json.dumps(data,sort_keys=True); actual=e(data); expected=completed[key]['output']
        assert json.dumps(data,sort_keys=True)==before
        normal=actual[2].get('normal') or {}
        comparisons.append(dict(key=key,reference_s=ref,exact=actual==expected,command=actual[:2],
            reason=normal.get('reason'),selector_reason=normal.get('selector_reason')))
        if ref in representatives:
            from arena_autonomy.follow_trace import restore_call
            _,raw,_=restore_call(data,e.motion)
            match=next((r for r in region_records if r['region']==normal.get('selector_region')),None)
            representative_geometry.append(dict(reference_s=ref,region=normal.get('selector_region'),
                geometry=geometry(match['scene'],raw,match['cfg'],match['context']) if match else None))
finally:
    e.close()

c=first_current; meta=c['compute'];plan=meta['plan']
request=current_request(plan['output'],plan['input'],meta['input'],
    retained=meta['route_check']['retained'],crop_current_selection=True)
active.update(stage='actual_current',reference_s=meta['input']['reference_s'],representative=True)
region_records.clear();cur=Evaluator('current',header)
try: current_output=cur(request)
finally:cur.close()
assert current_output==c['output'], 'actual first current exact mismatch'
from arena_autonomy.race_runtime_policy import released_relative
connection=dict(core_module=released_relative.core_policy.__name__,
    body_helper_module=body.bounded_body_nonreturn.__module__,
    adjacent_helper_module=body.bounded_adjacent_structure.__module__)
summary=dict(plans=len(comparisons),exact=sum(r['exact'] for r in comparisons),
    first_current_exact=current_output==c['output'],first_current_result=current_output,
    source_unchanged=all(hashlib.sha256((S/p).read_bytes()).hexdigest()==sha for p,sha in manifest.items()),
    trace_unchanged=trace_before==hashlib.sha256(TRACE.read_bytes()).hexdigest(),
    runtime_connections=connection,
    stall_body_failures=dict(collections.Counter(str(x['target_mps'])+':'+str(x['result'].get('blocked'))+':'+str((x['result'].get('proof') or {}).get('reason'))
        for x in events if x['kind']=='body_check' and x['stage']=='plan' and x['reference_s']>=19.22)))
for name,value in [('summary',summary),('comparisons',comparisons),('diagnostic_events',events),('representative_geometry',representative_geometry)]:
    (OUT/(name+'.json')).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='first_current_result'}),flush=True)
assert summary['plans']==summary['exact']==217 and summary['source_unchanged'] and summary['trace_unchanged']
