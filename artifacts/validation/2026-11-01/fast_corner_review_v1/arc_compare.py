"""회랑의 공통 호길이 비교를 출력에 연결하지 않은 별도 저장 입력 후보로 대조."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import sys
from zipfile import ZipFile,ZIP_DEFLATED

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import corridor_observer as module
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.local_follow import LocalFollow,FollowConfig,path_agrees_arc
from arena_autonomy.lidar_motion import MotionConfig

original=module.path_agrees


def alternative(candidate,reference,tolerance):
    return original(candidate,reference,tolerance) or path_agrees_arc(candidate,reference,tolerance)


def run(name):
    source=HERE.parent/name/'run1/follow_decisions.jsonl'
    counts=Counter();changes=[];footer=None;sequence=-1;digest=sha256()
    with source.open('rb') as stream:
        for line in stream:
            digest.update(line);event=json.loads(line)
            if footer is not None or event['seq']!=sequence+1:raise ValueError('trace ordering')
            sequence=event['seq'];data=event['data'];kind=event['kind']
            if sequence==0:
                if kind!='header' or data['schema']!=1:raise ValueError('solo header required')
                config=FollowConfig(**data['follow_config'])
                if not config.solo_route_only:raise ValueError('solo only experiment')
                old,new=LocalFollow(config),LocalFollow(config);motion=MotionConfig(**data['motion_config'])
            elif kind=='reset':old.reset();new.reset()
            elif kind=='footer':footer=data
            elif kind=='call':
                module.path_agrees=original;a=replay_call(old,data['input'],motion)
                if not equal_output(comparable_decision(freeze(a)),comparable_decision(data['output'])):
                    raise ValueError(f'baseline mismatch {name} seq={sequence}')
                module.path_agrees=alternative;b=replay_call(new,data['input'],motion)
                counts['baseline_matched']+=1
                changed=abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8
                counts['command_changed']+=int(changed)
                counts['detail_changed']+=int(not equal_output(comparable_decision(freeze(a)),comparable_decision(freeze(b))))
                counts['zero_released']+=int(b[0]>.001>=a[0]);counts['new_zero']+=int(a[0]>.001>=b[0])
                if changed:changes.append(dict(seq=sequence,t=data['input']['reference_s'],measured=data['input']['speed_mps'],
                    old_speed=a[0],new_speed=b[0],old_steering=a[1],new_steering=b[1],
                    old_reason=a[2].get('reason'),new_reason=b[2].get('reason'),
                    old_corridor=a[2].get('corridor_candidate'),new_corridor=b[2].get('corridor_candidate')))
            else:raise ValueError('unknown event')
    if (footer is None or footer['written']!=sequence or footer['accepted']!=sequence
            or footer['worker_alive'] is not False or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):
        raise ValueError('incomplete trace')
    result=dict(case=name,counts=dict(counts),changes=changes,input_sha256=digest.hexdigest(),
                input_unchanged=digest.hexdigest()==sha256(source.read_bytes()).hexdigest())
    with (HERE/f'arc_compare_{name}.json').open('x',encoding='utf-8') as stream:json.dump(freeze(result),stream,ensure_ascii=False,indent=2)
    print(json.dumps(dict(case=name,counts=dict(counts)),ensure_ascii=False),flush=True)
    if not result['input_unchanged']:raise ValueError('input changed')
    return {k:v for k,v in result.items() if k!='changes'}


if __name__=='__main__':
    files=[Path(__file__).resolve(),*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    with ZipFile(HERE/'arc_compare_source.zip','x',ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,p.relative_to(ROOT).as_posix())
    try:results=[run(name) for name in ('solo_headway_v1','solo_route_v1')]
    finally:module.path_agrees=original
    unchanged=all(sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    output=dict(scope='별도 저장 입력 실험. 제어 소스 변경·ROS 연결·실제 새 궤적은 없음.',
                source_sha256=hashes,source_unchanged=unchanged,results=results)
    with (HERE/'arc_compare_summary.json').open('x',encoding='utf-8') as stream:json.dump(output,stream,ensure_ascii=False,indent=2)
    if not unchanged:raise ValueError('source changed')
