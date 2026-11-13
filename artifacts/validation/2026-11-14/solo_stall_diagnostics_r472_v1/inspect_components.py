"""정체 대표 3개의 같은 .15 query 성분만 진단한다. 허가/정책 변경 없음."""
from pathlib import Path
import collections, hashlib, json, math, sys
import numpy as np
A=Path(__file__).resolve().parent;S=A/'stage'
sys.path[:0]=[str(S/'src/arena_autonomy'),str(S/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.known_structure import parse_sectors,sector_rays,classify
from arena_autonomy.scan_input import scan_angles
from arena_autonomy.rear_fringe_trial import _query_brackets
from arena_autonomy import pass_release_trial as ps
T=A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'
before=hashlib.sha256(T.read_bytes()).hexdigest()
records=[json.loads(l) for l in T.open(encoding='utf-8')]
header=next(r['data']['header'] for r in records if r['kind']=='runtime_evaluator_header')
submitted=[r['data'] for r in records if r['kind']=='runtime_plan_submitted' and r['data']['input']['reference_s']>=19.22]
events=json.loads((A/'diagnostic_v1/diagnostic_events.json').read_text(encoding='utf-8'))
e=Evaluator('current',header);cfg=e.follow.config;results=[]
try:
    for item in (submitted[0],submitted[len(submitted)//2],submitted[-1]):
        data=item['input'];ref=data['reference_s'];info,raw,history=restore_call(data,e.motion)
        scene=motion_scene(raw,history,ref,timing_verified=info['timing_verified'],epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        event=next(r for r in events if r['kind']=='body_check' and r['reference_s']==ref and r['target_mps']==.15)
        ctx={};region=ps.motion_region(scene,event['speed_mps'],.15,event['steering_rad'],event['previous_rad'],event['dt_s'],cfg,ps.lateral_config(cfg),context=ctx)
        assert freeze(region)==event['result']['region']
        q=ctx['queries'][~ctx['free'] & ~ctx['masked']];pad=ctx['padding'];rays=scene.rays
        origins=np.asarray([r.ray_origin_xy_m for r in rays]);directions=np.asarray([r.direction_xy for r in rays]);status=np.asarray([r.status for r in rays])
        depth=np.asarray([math.dist(r.ray_origin_xy_m,r.hit_xy_m) if r.status=='return' else -1. for r in rays])
        angles=np.asarray(scan_angles(raw));values=np.asarray(raw.ranges,dtype=float)
        sectors=parse_sectors(cfg.known_structure_sectors_json)
        members=[set(x.tolist()) for x in sector_rays(0.,0.,len(rays),sectors,source_angles=angles)]
        _,_,classified=classify(raw,sectors)
        touched=set()
        for _,_,lo,hi in _query_brackets(q,origins,directions):
            for i in (lo,hi):
                if status[i]=='unknown':touched.add(i)
        components={}
        for i in sorted(touched):
            a=b=i
            while a>0 and status[a-1] in ('unknown','masked'):a-=1
            while b+1<len(rays) and status[b+1] in ('unknown','masked'):b+=1
            if (a,b) in components:continue
            masked=set(j for j in range(a,b+1) if status[j]=='masked')
            unknown=set(range(a,b+1))-masked
            flanks=[a-1,b+1];positions=origins[a-1:b+2]
            angle=float(np.ptp(np.unwrap(np.arctan2(directions[a-1:b+2,1],directions[a-1:b+2,0]))))
            span=float(np.max(np.linalg.norm(positions[:,None]-positions[None],axis=2)))
            reach=float(np.max(np.linalg.norm(q[:,None,:]-origins[None,a-1:b+2,:],axis=2)))
            comparisons=[dict(sector=j,declared_indices=sorted(m),intersection=sorted(masked&m),
                declared_missing_from_component=sorted(m-masked),masked_outside_sector=sorted(masked-m),
                exact=masked==m) for j,m in enumerate(members)]
            unknown_spans=[]
            for j in sorted(unknown):
                if j-1 in unknown:continue
                end=j
                while end+1 in unknown:end+=1
                local=directions[[j-1,end+1]]
                gap=abs(math.atan2(float(local[0,0]*local[1,1]-local[0,1]*local[1,0]),float(np.dot(local[0],local[1]))))
                unknown_spans.append(dict(first=j,last=end,gap_deg=math.degrees(gap)))
            components[a,b]=dict(first=a,last=b,masked_indices=sorted(masked),unknown_indices=sorted(unknown),
                mask_outside_all_declared=sorted(masked-set().union(*members)),sector_comparisons=comparisons,
                source_angles_deg=[math.degrees(float(angles[a])),math.degrees(float(angles[b]))],
                flank_indices=flanks,flank_status=status[flanks].tolist(),flank_raw_ranges=freeze(values[flanks].tolist()),
                flank_depth_m=depth[flanks].tolist(),flank_padding_m=pad[flanks].tolist(),
                flank_min_query_margin_m=[float(depth[j]-pad[j]-np.linalg.norm(q-origins[j],axis=1).max()) for j in flanks],
                query_reach_m=reach,origin_span_m=span,combined_angle_deg=math.degrees(angle),
                unknown_spans=unknown_spans,
                old_adjacent_chord_with_padding_m=2*(reach+float(pad.max()))*math.sin(angle/2),
                geometry_shadow_width_without_padding_plus_origin_span_m=2*reach*math.sin(angle/2)+span,
                width_formula_scope='diagnostic of whole unknown+masked component; not an accepted proof',
                raw_unknown_values=freeze(values[sorted(unknown)].tolist()),
                raw_masked_values=freeze(values[sorted(masked)].tolist()))
        results.append(dict(reference_s=ref,region_exact=True,region=region,
            sector_classification=freeze(classified),components=list(components.values())))
finally:e.close()
assert before==hashlib.sha256(T.read_bytes()).hexdigest()
(A/'component_evidence.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n',encoding='utf-8')
for row in results:
    print(json.dumps({'ref':row['reference_s'],'components':[{k:v for k,v in c.items() if k not in ('raw_masked_values','raw_unknown_values','sector_comparisons')} for c in row['components']]}))
