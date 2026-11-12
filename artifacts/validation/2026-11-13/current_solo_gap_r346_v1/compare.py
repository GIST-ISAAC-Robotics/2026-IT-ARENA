"""r338/r344 전체 현재 입력과 과거 교통 보류 표본을 같은 입력으로 대조."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r346_helpers',A.parent/'current_input_jetson_r296_v1/measure_cost.py')
old=importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
spec=importlib.util.spec_from_file_location('r346_candidate',A/'candidate.py')
new=importlib.util.module_from_spec(spec)
spec.loader.exec_module(new)


def main():
    assert not (A/'report.json').exists()
    old.configure('native_v1')
    records=[]
    inputs=[]
    for label,folder in [('r338','published_route_drive_r338_v1'),('r344','current_solo_low_drive_r344_v1')]:
        source=A.parent/folder/'run1/follow_decisions.jsonl'
        inputs.append(source)
        with source.open() as stream:
            header=json.loads(next(stream))['data']
            records.extend((label,header,e['data']) for e in map(json.loads,stream)
                           if e['kind']=='call' and 'current_rebase' in e['data'])
    holdout=A.parent/'current_input_jetson_r296_v1'
    inputs.extend([holdout/'headers.json',holdout/'selected_inputs.json'])
    headers=json.loads((holdout/'headers.json').read_bytes())
    records.extend(('holdout_'+e['source'],headers[e['source']],e['input'])
                   for e in json.loads((holdout/'selected_inputs.json').read_bytes()))
    rows,answers=[],[]
    configs={}
    for index,(label,h,c) in enumerate(records):
        if label not in configs:
            configs[label]=(old.MotionConfig(**h['motion_config']),old.FollowConfig(**h['follow_config']))
        motion,cfg=configs[label]
        current=c['current_rebase']['input']
        d,scan,history=old.restore_call(current,motion)
        scene=old.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
                               epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        outputs,costs={},{}
        order=[('before',new.prior),('after',new)]
        if index%2:order.reverse()
        for mode,module in order:
            t=time.perf_counter_ns()
            outputs[mode]=old.freeze(module.rebase(c['output'],old.plan_reference(c['input']),current,
                                                   scene,history,cfg,require_current_region=True))
            costs[mode]=(time.perf_counter_ns()-t)/1e6
        before,after=outputs['before'],outputs['after']
        changed=not old.equal_output(before,after)
        retry=after.get('current_solo_gap_retry',{})
        if label=='r344':
            # 기록의 ROS 인계 후 감속 상한은 순수 rebase의 밖에서 붙는다.
            # 해당 단계만 원 기록대로 재현하여 원판 전체를 대조한다.
            expected=c['current_rebase']['output']
            cap=expected.get('new_plan_deceleration_cap')
            replay=before
            if cap is not None:
                assert old.equal_output(before['speed_mps'],cap['checked_target_mps'])
                assert cap['steering_changed'] is False and cap['current_check_reused_without_timestamp_renewal'] is True
                assert cap['applied']==(min(before['speed_mps'],cap['speed_mps'])<before['speed_mps'])
                replay={**before,'speed_mps':min(before['speed_mps'],cap['speed_mps']),
                        'new_plan_deceleration_cap':cap}
            if not old.equal_output(replay,expected):
                (A/'mismatch_next.json').write_text(json.dumps(dict(index=index,record=c,before=replay,after=after),indent=2))
            assert old.equal_output(replay,expected),(index,'original mismatch')
        if retry.get('applied'):
            assert before['speed_mps']==0 and 0<after['speed_mps']<=.15
            assert after['steering_rad']==before['steering_rad'] and after['reference_s']==before['reference_s']
            assert after['current_route_proof']['accepted']
            assert after['current_route_proof']['region']['unobserved']>0
            assert not after['current_route_proof']['unknown_filled']
        elif changed:
            assert old.equal_output(before,{k:v for k,v in after.items() if k!='current_solo_gap_retry'})
        rows.append(dict(label=label,index=index,reference_s=d['reference_s'],speed_mps=d['speed_mps'],
            changed=changed,applied=retry.get('applied',False),retry=bool(retry),
            before_speed=before['speed_mps'],after_speed=after['speed_mps'],
            before_reason=before['reason'],after_reason=after['reason'],times_ms=costs))
        answers.append(outputs)
    summary={label:dict(count=sum(r['label']==label for r in rows),
        changed=sum(r['changed'] for r in rows if r['label']==label),
        applied=sum(r['applied'] for r in rows if r['label']==label)) for label in configs}
    sources={}
    for module in list(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_file() and p.is_relative_to(ROOT) and p.suffix in ('.py','.so'):
                sources[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in sources:archive.write(ROOT/path,path)
    report=dict(scope='fixed sensor inputs; no driving or hardware proof; default unchanged',summary=summary,rows=rows,
        source_hashes=sources,input_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
    (A/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (A/'answers.json').write_text(json.dumps(answers,separators=(',',':'))+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
