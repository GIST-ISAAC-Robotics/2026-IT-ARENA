"""기존 중앙 정체의 센서 입력에서만 외측 재계획과 독립 보호를 비교한다."""
from collections import Counter
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from candidate import SteeringRecoveryGuide
from arena_autonomy.bypass_entry_trial import EntryGuide,EntryConfig
from arena_autonomy.follow_trace import restore_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
import probe_bypass_entry_sequence as probe

OLD=BASE.parent/'g2_center_offsets_v1'
spec=importlib.util.spec_from_file_location('previous_offsets_runner',OLD/'run.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
INPUT=OLD/'comparison1/existing_027/input_output.jsonl'


def main():
    output=BASE/'shadow1';output.mkdir(exist_ok=False)
    prior=json.loads((OLD/'comparison1/report.json').read_text())
    if any(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h for p,h in prior['source_sha256'].items()):
        raise ValueError('original recorded source changed')
    before=hashlib.sha256(INPUT.read_bytes()).hexdigest()
    own={p.relative_to(ROOT).as_posix():p.read_bytes() for p in BASE.glob('*.py')}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for n,v in own.items():archive.writestr(n,v)
    counts=Counter();replans=[];rows=[];cost=[];seq=-1;footer=False
    for line in INPUT.open(encoding='utf-8'):
        event=json.loads(line)
        if footer or event['seq']!=seq+1:raise ValueError('record sequence')
        seq=event['seq'];d=event['data']
        if event['kind']=='header':
            if d['schema']!=907:raise ValueError('expected recorded guide')
            config=FollowConfig(**d['config']);motion=MotionConfig(**d['motion']);duration=d['duration_s']
            kwargs=dict(d['entry']);kwargs['offsets_m']=tuple(kwargs['offsets_m']);kwargs['transitions_m']=tuple(kwargs['transitions_m'])
            original=EntryGuide(config,EntryConfig(**kwargs),**d['guide_kwargs'])
            candidate=SteeringRecoveryGuide(config,EntryConfig(**kwargs),**d['guide_kwargs'])
            guard=probe.ShadowSafety(RecoveryPolicy(True,True,True),config,30.)
        elif event['kind']=='call':
            inp,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],
                epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
            args=(scene,history,inp['speed_mps'],inp['previous_steering_rad'],inp['dt_s'])
            baseline=freeze(original.evaluate(*args));counts['calls']+=1
            if not equal_output(old.comparable(baseline),old.comparable(d['output'])):
                raise ValueError(f'original decision changed at {seq}')
            counts['baseline_matched']+=1
            started=time.perf_counter();result=freeze(candidate.evaluate(*args));cost.append((time.perf_counter()-started)*1000)
            selected=result['selected'];stopping=inp['reference_s']>=.5+duration
            command=(0.,inp['previous_steering_rad']) if stopping or selected is None else (selected['speed_mps'],selected['steering_rad'])
            safe,status=guard.step(d['input'],motion,command)
            if not 0<=safe[0]<=command[0]+1e-12:raise ValueError('independent protection increased speed')
            counts['command_changed']+=not equal_output(list(command),d['command'])
            counts['guarded_zero_to_positive']+=d['safe'][0]==0<safe[0]
            counts['guarded_positive_to_zero']+=safe[0]==0<d['safe'][0]
            diag=result.get('steering_recovery',{})
            if diag.get('applied'):counts[diag['kind']]+=1
            if diag.get('applied'):replans.append(dict(reference_s=scene.t_ref_s,detail=diag,selected=selected))
            rows.append(dict(reference_s=scene.t_ref_s,original_command=d['command'],candidate_command=list(command),
                original_safe=d['safe'],candidate_safe=list(safe),guard_reason=status['reason'],
                candidate_reason=result['reason'],steering_recovery=diag,processing_ms=cost[-1]))
        elif event['kind']=='footer':
            footer=True
            if counts['calls']!=d['calls']:raise ValueError('footer count')
        else:raise ValueError('unexpected event')
    if not footer:raise ValueError('missing footer')
    (output/'rows.json').write_text(json.dumps(rows,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    ordered=sorted(cost)
    report=dict(counts=dict(counts),replans=replans,input_sha256=before,
        input_unchanged=hashlib.sha256(INPUT.read_bytes()).hexdigest()==before,
        baseline_source_matched=True,source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in own.items()},
        source_unchanged=all((ROOT/n).read_bytes()==v for n,v in own.items()),
        processing_ms=dict(median=ordered[len(ordered)//2],p95=ordered[int(.95*(len(ordered)-1))],max=max(cost)),
        scope='counterfactual on preserved input; original actuation history retained; not new driving/clearance proof')
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    if not report['input_unchanged'] or not report['source_unchanged']:raise ValueError('changed evidence')


if __name__=='__main__':main()
