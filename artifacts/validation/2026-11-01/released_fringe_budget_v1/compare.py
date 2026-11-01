"""후반부 1 mm 후보를 저장 입력에서 대조한다. 과거 기본 출력을 먼저 재현한다."""
from collections import Counter
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
from zipfile import ZipFile,ZIP_DEFLATED

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import released_rear_sweep as module
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow,FollowConfig

with ZipFile(HERE/'before.zip') as archive:
    old_code=archive.read('src/arena_autonomy/arena_autonomy/released_rear_sweep.py')
old_globals={}
exec(compile(old_code,'before/released_rear_sweep.py','exec'),old_globals)
old_function=old_globals['bounded_released_rear_sweep'];new_function=module.bounded_released_rear_sweep


def controller(data):
    config=FollowConfig(**data['follow_config'])
    if data['schema']==1 and data['scope']=='g1_decision_stage_only':return LocalFollow(config)
    if (data['schema']==5 and data['scope']=='g1_g2_handoff_decision_stage_only'
            and data['controller_mode']=='g1_g2_handoff_trial'):
        return BypassHandoffFollow(config,HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
    raise ValueError('unsupported comparison trace')


def run(path):
    counts=Counter();reasons=Counter();changes=[];hasher=sha256();footer=None;count=0
    with path.open('rb') as stream:
        for line in stream:
            hasher.update(line);event=json.loads(line);data=event['data'];kind=event['kind']
            if footer is not None or event['seq']!=count:raise ValueError('trace order/footer')
            count+=1
            if count==1:
                if kind!='header':raise ValueError('header required')
                baseline=controller(data);candidate=controller(data);motion=MotionConfig(**data['motion_config'])
            elif kind=='reset':baseline.reset();candidate.reset()
            elif kind=='footer':footer=data
            elif kind=='call':
                inp=data['input'];module.bounded_released_rear_sweep=old_function
                old=replay_call(baseline,inp,motion)
                if not equal_output(comparable_decision(freeze(old)),comparable_decision(data['output'])):
                    raise ValueError(f'old output mismatch {path} seq={event["seq"]}')
                counts['baseline_matched']+=1
                module.bounded_released_rear_sweep=new_function
                new=replay_call(candidate,inp,motion)
                normal=new[2].get('normal',new[2]);risk=normal.get('selector_released_rear_sweep') or {}
                reasons[risk.get('policy','unused')+':'+str(risk.get('accepted',False))]+=1
                command=abs(old[0]-new[0])>1e-8 or abs(old[1]-new[1])>1e-8
                old_mode=old[2].get('handoff_state');new_mode=new[2].get('handoff_state')
                counts['detail_changed']+=not equal_output(comparable_decision(freeze(old)),comparable_decision(freeze(new)))
                counts['command_changed']+=command
                counts['state_changed']+=old_mode!=new_mode
                counts['new_zero']+=old[0]>.001>=new[0]
                counts['zero_released']+=new[0]>.001>=old[0]
                if command or old_mode!=new_mode:
                    changes.append(dict(seq=event['seq'],reference_s=inp['reference_s'],measured_mps=inp['speed_mps'],
                        old_speed=old[0],new_speed=new[0],old_steering=old[1],new_steering=new[1],
                        old_state=old_mode,new_state=new_mode,reason=normal.get('selector_reason'),risk=risk))
            else:raise ValueError('unknown event')
    if (footer is None or footer['written']!=count-1 or footer['accepted']!=count-1
            or footer['worker_alive'] is not False or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):
        raise ValueError('incomplete trace')
    digest=hasher.hexdigest()
    if digest!=sha256(path.read_bytes()).hexdigest():raise ValueError('trace changed')
    return dict(path=path.relative_to(ROOT).as_posix(),input_sha256=digest,counts={k:int(v) for k,v in counts.items()},
                policies=dict(reasons),changes=changes,scope='고정 입력 대조. 새 폐루프 궤적을 재현하지 않는다.')


def write_json(path,value):
    payload=json.dumps(freeze(value),ensure_ascii=False,indent=2,allow_nan=False)
    with path.open('x',encoding='utf-8') as f:f.write(payload)


if __name__=='__main__':
    cases=('corner_parallel_v2/run1','solo_route_v1/run1',
           'follow_continuity_v1/fast_stop_paced','race_distance_v2/run1')
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',choices=cases,action='append')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    if not output.is_relative_to(HERE) or output==HERE:raise ValueError('new result directory under this trial required')
    output.mkdir(exist_ok=False)
    files=[p for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')]+[Path(__file__).resolve()]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    with ZipFile(output/'source_snapshot.zip','x',ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,p.relative_to(ROOT).as_posix())
    results=[]
    try:
        for index,name in enumerate(args.case or cases):
            result=run(HERE.parent/name/'follow_decisions.jsonl')
            checkpoint=output/f'trace_{index+1}.json'
            write_json(checkpoint,result)
            brief={k:v for k,v in result.items() if k!='changes'}
            brief['checkpoint']=checkpoint.name;results.append(brief)
            print(json.dumps(freeze(dict(path=name,counts=result['counts'],policies=result['policies'])),ensure_ascii=False),flush=True)
    finally:module.bounded_released_rear_sweep=new_function
    unchanged=all(sha256((ROOT/name).read_bytes()).hexdigest()==value for name,value in hashes.items())
    write_json(output/'comparison.json',dict(source_sha256=hashes,source_unchanged=unchanged,results=results))
    if not unchanged:raise ValueError('source changed')
