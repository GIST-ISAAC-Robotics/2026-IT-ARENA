import hashlib,json,sys
from pathlib import Path
from collections import Counter
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from candidate import trial
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
cases={'contact':HERE.parent/'solo_contact_diagnosis_v1','r14':HERE.parent/'oblique_gap_connected_v1','straight':ROOT/'artifacts/validation/2026-11-07/solo_straight_newcourse_v1'}
name=sys.argv[1];run=cases[name]/'run1';source=run/'follow_decisions.jsonl';trace=json.loads((run/'trace.json').read_text());ti=0
rows=[];n=0;found=0;cut_before=Counter();early=[];speedopp=0;kept=0
with source.open() as f:
    header=json.loads(next(f))['data'];cfg=FollowConfig(**header['follow_config']);mc=MotionConfig(**header['motion_config'])
    for line in f:
        e=json.loads(line)
        if e['kind']!='call':continue
        data=e['data'];o=data['output'];dt=data['input'];n+=1
        when=dt['reference_s'];assert when is not None
        while ti+1<len(trace) and trace[ti+1]['time']<=when:ti+=1
        ds=trace[ti]['progress_m']
        if o[0]<=1.2 or o[2].get('reason')!='local_follow':continue
        di,scan,h=restore_call(dt,mc)
        try:scene=motion_scene(scan,h,di['reference_s'],timing_verified=di['timing_verified'],epoch=di['epoch'],rear_half_angle_deg=di['rear_half_angle_deg'])
        except (ValueError,MotionUnavailable):continue
        speed,detail=trial(o,scene,cfg);found+=detail is not None
        assert 1.2-1e-9<=speed<=o[0]+1e-9
        if o[0]>=10/3.6:speedopp+=1;kept+=speed>=10/3.6
        if speed<o[0]-1e-6:
            row=dict(t=when,progress_m=ds,original_mps=o[0],candidate_mps=speed,**detail);rows.append(row)
            if 30<=ds<=35:early.append(row)
result=dict(case=name,scope='fixed inputs, no closed-loop vehicle or real braking proof',calls=n,crossing_detected=found,changed=len(rows),target_10kmh_calls=speedopp,retained_10kmh_calls=kept,first_target_region=early[:1],target_region_max_reduction=max([r['original_mps']-r['candidate_mps'] for r in early] or [0.]),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),changes=rows)
(HERE/f'{name}.json').write_text(json.dumps(result,indent=1)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='changes'}))
