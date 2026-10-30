"""후보 B의 입력/출력 기록과 독립 재생. schema 910은 오프라인 전용이며 운영 schema5가 아니다."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe
from arena_autonomy.bypass_entry_trial import EntryConfig, EntryGuide
from candidate import CorridorGuide, CorridorPolicy
from arena_autonomy.follow_trace import equal_output, freeze, restore_call, snapshot_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy

CASES=[dict(name='existing_center_s17',variant='existing',target_y=0.,seed=17),
    dict(name='B_center_s17',variant='B',target_y=0.,seed=17),
    dict(name='B_center_s23',variant='B',target_y=0.,seed=23),
    dict(name='B_edge_right_s17',variant='B',target_y=-.15,seed=17)]
POLICY=CorridorPolicy(surface=True,plan=True,onset=True)


def comparable(value):
    if isinstance(value,dict):
        return {k:comparable(v) for k,v in value.items() if k!='processing_ms'}
    if isinstance(value,list):
        return [comparable(v) for v in value]
    return value


def make_policy(raw):
    return CorridorPolicy(**{**raw,'transitions_m':tuple(raw['transitions_m'])})


def replay(path):
    last=-1;footer=False;calls=matched=safe_matched=0;differences=[]
    for line in path.open(encoding='utf-8'):
        event=json.loads(line)
        if footer or event['seq']!=last+1:
            raise ValueError('record sequence')
        last=event['seq'];d=event['data']
        if event['kind']=='header':
            if d['schema']!=910 or d['scope']!='offline_g2_corridor_guide_and_guard':
                raise ValueError('offline scope')
            if d['candidate_sha256']!=hashlib.sha256((BASE/'candidate.py').read_bytes()).hexdigest():
                raise ValueError('candidate hash mismatch')
            config=FollowConfig(**d['config']);entry=dict(d['entry'])
            entry['offsets_m']=tuple(entry['offsets_m']);entry['transitions_m']=tuple(entry['transitions_m'])
            if d['variant']=='existing':
                guide=EntryGuide(config,EntryConfig(**entry),**d['guide_kwargs'])
            else:
                guide=CorridorGuide(config,EntryConfig(**entry),corridor_policy=make_policy(d['policy']),**d['guide_kwargs'])
            motion=MotionConfig(**d['motion']);duration=d['duration_s']
            guard=probe.ShadowSafety(RecoveryPolicy(True,True,True),config,30.)
        elif event['kind']=='call':
            raw,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,raw['reference_s'],timing_verified=raw['timing_verified'],
                epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
            output=freeze(guide.evaluate(scene,history,raw['speed_mps'],raw['previous_steering_rad'],raw['dt_s']))
            same=equal_output(comparable(output),comparable(d['output']))
            calls+=1;matched+=same
            if not same and len(differences)<5:
                differences.append(event['seq'])
            chosen=output['selected']
            command=((0.,raw['previous_steering_rad']) if raw['reference_s']>=.5+duration or chosen is None
                     else (chosen['speed_mps'],chosen['steering_rad']))
            safe,status=guard.step(d['input'],motion,command)
            safe_matched+=(equal_output(list(command),d['command']) and equal_output(list(safe),d['safe'])
                           and status['reason']==d['safety_reason'])
        elif event['kind']=='footer':
            footer=True
            if d['calls']!=calls:
                raise ValueError('call count')
        else:
            raise ValueError('unknown event')
    if not footer:
        raise ValueError('missing footer')
    return dict(calls=calls,guide_matched=matched,guard_matched=safe_matched,differences=differences,
        passed=matched==safe_matched==calls and calls>0)


def rollout(case,out):
    decisions=[];inputs=[];construction={}
    original_snapshot=probe.snapshot_call
    base=EntryGuide if case['variant']=='existing' else CorridorGuide
    class Recorded(base):
        def __init__(self,config,entry,**kwargs):
            construction.update(config=asdict(config),entry=asdict(entry),guide_kwargs=dict(kwargs))
            if base is CorridorGuide:
                kwargs['corridor_policy']=POLICY
            super().__init__(config,entry,**kwargs)
        def evaluate(self,*args,**kwargs):
            output=super().evaluate(*args,**kwargs);decisions.append(freeze(output));return output
    def snapshot(*args,**kwargs):
        value=original_snapshot(*args,**kwargs);inputs.append(value);return value
    began=time.perf_counter()
    with patch.object(probe,'EntryGuide',Recorded),patch.object(probe,'snapshot_call',snapshot):
        result=probe.rollout(case['target_y'],False,20.,True,True,True,True,'combined',case['seed'],True,True,False)
    rollout_s=time.perf_counter()-began
    if len(inputs)!=len(decisions) or len(inputs)!=len(result['records']):
        raise ValueError('capture length')
    trace=out/'input_output.jsonl'
    with trace.open('x',encoding='utf-8') as stream:
        def emit(seq,kind,data):
            stream.write(json.dumps(dict(seq=seq,kind=kind,data=freeze(data)),allow_nan=False)+'\n')
        emit(0,'header',dict(schema=910,scope='offline_g2_corridor_guide_and_guard',variant=case['variant'],duration_s=20.,
            motion=result['motion_config'],candidate_sha256=hashlib.sha256((BASE/'candidate.py').read_bytes()).hexdigest(),
            policy=asdict(POLICY),**construction))
        for i,(inp,output,record) in enumerate(zip(inputs,decisions,result['records']),1):
            emit(i,'call',dict(input=inp,output=output,command=record['command'],safe=record['safe'],safety_reason=record['safety_reason']))
        emit(len(inputs)+1,'footer',dict(calls=len(inputs)))
    (out/'records.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    records=result['records']
    report={k:v for k,v in result.items() if k!='records'}
    centers=[r['evaluation']['center_pose'] for r in records]
    report.update(case=case,rollout_seconds=rollout_s,
        passed_target_t=next((r['t'] for r,c in zip(records,centers) if c[0]-.1>1.3),None),
        rejoin_t=next((r['t'] for r in records if r['entry_phase']=='rejoin'),None),
        centered_t=next((r['t'] for r in records if r['entry_phase']=='centered'),None),
        first_bypass=next((dict(t=r['t'],selected=r['selected']) for r in records if r['selected'] and r['selected']['offset_m']),None),
        safety_reduced_ticks=sum(r['safe'][0]<r['command'][0]-1e-12 for r in records),
        target_gap_min_m=min(r['evaluation']['target_body_gap_m'] for r in records),
        processing_ms=dict(mean=sum(r['processing_ms'] for r in records)/len(records),
            p95=sorted(r['processing_ms'] for r in records)[int(.95*len(records))],maximum=max(r['processing_ms'] for r in records)),
        replay=replay(trace),trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest())
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


if __name__=='__main__':
    paths=list((ROOT/'src').rglob('*.py'))+[Path(__file__),BASE/'candidate.py',BASE/'test_candidate.py']+list((ROOT/'scripts').glob('*bypass*.py'))
    paths+=[ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    out=BASE/'trace2';out.mkdir(exist_ok=False)
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:
            archive.write(p,p.relative_to(ROOT))
    reports=[]
    for case in CASES:
        target=out/case['name'];target.mkdir()
        r=rollout(case,target);reports.append(r)
        print(json.dumps({k:r[k] for k in ('final_forward_progress_m','lateral_displacement_m','contact_samples','minimum_body_gap_m',
            'maximum_wheels_touching_grass','stop_command_all_zero','final_speed_mps','passed_target_t','rejoin_t','centered_t','replay')}|{'name':case['name']}),flush=True)
        if not r['replay']['passed']:
            raise ValueError('record replay failed')
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (out/'report.json').write_text(json.dumps(dict(cases=reports,source_sha256=hashes,source_unchanged=unchanged),indent=2)+'\n',encoding='utf-8')
    if not unchanged:
        raise ValueError('source changed')
