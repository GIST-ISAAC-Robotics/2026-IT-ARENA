"""등록 편이의 빈 구간 가설을 같은 중앙 정지차 합성으로 한 번 대조한다."""
from dataclasses import asdict,replace
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe
from arena_autonomy.bypass_entry_trial import EntryConfig,EntryGuide
from arena_autonomy.follow_trace import freeze,restore_call,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


def comparable(value):
    if isinstance(value,dict):return {k:comparable(v) for k,v in value.items() if k!='processing_ms'}
    if isinstance(value,list):return [comparable(v) for v in value]
    return value


def replay(path):
    last=-1;footer=False;calls=matched=safe_matched=0;differences=[]
    for line in path.open(encoding='utf-8'):
        event=json.loads(line)
        if footer or event['seq']!=last+1:raise ValueError('record sequence')
        last=event['seq'];d=event['data']
        if event['kind']=='header':
            if d['schema']!=907 or d['scope']!='offline_g2_offset_guide_and_guard':raise ValueError('offline scope')
            config=FollowConfig(**d['config']);entry=dict(d['entry'])
            entry['offsets_m']=tuple(entry['offsets_m']);entry['transitions_m']=tuple(entry['transitions_m'])
            guide=EntryGuide(config,EntryConfig(**entry),**d['guide_kwargs'])
            motion=MotionConfig(**d['motion']);duration=d['duration_s']
            guard=probe.ShadowSafety(RecoveryPolicy(True,True,True),config,30.)
        elif event['kind']=='call':
            raw,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,raw['reference_s'],timing_verified=raw['timing_verified'],
                epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
            output=freeze(guide.evaluate(scene,history,raw['speed_mps'],raw['previous_steering_rad'],raw['dt_s']))
            same=equal_output(comparable(output),comparable(d['output']))
            calls+=1;matched+=same
            if not same and len(differences)<5:differences.append(event['seq'])
            chosen=output['selected']
            command=(0.,raw['previous_steering_rad']) if raw['reference_s']>=.5+duration or chosen is None else (chosen['speed_mps'],chosen['steering_rad'])
            safe,status=guard.step(d['input'],motion,command)
            safe_matched+=equal_output(list(command),d['command']) and equal_output(list(safe),d['safe']) and status['reason']==d['safety_reason']
        elif event['kind']=='footer':
            footer=True
            if d['calls']!=calls:raise ValueError('call count')
        else:raise ValueError('unknown event')
    if not footer:raise ValueError('missing footer')
    return dict(calls=calls,guide_matched=matched,guard_matched=safe_matched,differences=differences,
        passed=matched==safe_matched==calls and calls>0)


def rollout(variant,out):
    decisions=[];inputs=[];construction={}
    original_snapshot=probe.snapshot_call
    class RecordedGuide(EntryGuide):
        def __init__(self,config,entry,**kwargs):
            construction.update(config=asdict(config),entry=asdict(entry),guide_kwargs=kwargs)
            super().__init__(config,entry,**kwargs)
        def evaluate(self,*args,**kwargs):
            output=super().evaluate(*args,**kwargs);decisions.append(freeze(output));return output
    def entry_config(*args,**kwargs):
        c=EntryConfig(*args,**kwargs)
        return (replace(c,offsets_m=tuple(.26 if x==.27 else -.26 if x==-.27 else x for x in c.offsets_m))
                if variant=='observed_gap_026' else c)
    def snapshot(*args,**kwargs):
        value=original_snapshot(*args,**kwargs);inputs.append(value);return value
    with patch.object(probe,'EntryConfig',entry_config),patch.object(probe,'EntryGuide',RecordedGuide),patch.object(probe,'snapshot_call',snapshot):
        result=probe.rollout(0.,False,10.,True,True,True,True,'combined',17,True,True,False)
    if len(inputs)!=len(decisions) or len(inputs)!=len(result['records']):raise ValueError('capture length')
    trace=out/'input_output.jsonl'
    with trace.open('x',encoding='utf-8') as stream:
        def emit(seq,kind,data):stream.write(json.dumps(dict(seq=seq,kind=kind,data=freeze(data)),allow_nan=False)+'\n')
        emit(0,'header',dict(schema=907,scope='offline_g2_offset_guide_and_guard',variant=variant,duration_s=10.,
            motion=result['motion_config'],**construction))
        for i,(inp,output,record) in enumerate(zip(inputs,decisions,result['records']),1):
            emit(i,'call',dict(input=inp,output=output,command=record['command'],safe=record['safe'],safety_reason=record['safety_reason']))
        emit(len(inputs)+1,'footer',dict(calls=len(inputs)))
    (out/'records.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    report={k:v for k,v in result.items() if k!='records'}
    report['variant']=variant;report['entry_config']=construction['entry'];report['replay']=replay(trace)
    report['trace_sha256']=hashlib.sha256(trace.read_bytes()).hexdigest()
    report['first_bypass']=next((dict(t=r['t'],selected=r['selected']) for r in result['records']
        if r['selected'] is not None and r['selected']['offset_m']),None)
    report['last_entry_phase']=result['records'][-1]['entry_phase']
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


if __name__=='__main__':
    paths=list((ROOT/'src').rglob('*.py'))+[Path(__file__)]+list((ROOT/'scripts').glob('*bypass*.py'))
    paths += [ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    out=BASE/'comparison1';out.mkdir(exist_ok=False)
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(p,p.relative_to(ROOT))
    reports=[]
    for variant in ('existing_027','observed_gap_026'):
        target=out/variant;target.mkdir();r=rollout(variant,target);reports.append(r)
        print(json.dumps({k:r[k] for k in ('variant','final_forward_progress_m','lateral_displacement_m',
            'contact_samples','minimum_body_gap_m','maximum_wheels_touching_grass','stop_command_all_zero',
            'final_speed_mps','first_bypass','last_entry_phase','replay')}),flush=True)
        if not r['replay']['passed']:raise ValueError('record replay failed')
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (out/'report.json').write_text(json.dumps(dict(cases=reports,source_sha256=hashes,source_unchanged=unchanged),indent=2)+'\n',encoding='utf-8')
    if not unchanged:raise ValueError('source changed')
