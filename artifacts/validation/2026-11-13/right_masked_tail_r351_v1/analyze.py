"""r349 오른쪽 자기 가림 정체의 같은 입력/조향에서 속도만 낮춰 대조한다."""
import hashlib
import importlib.util
import json
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result

old=module('r351_replay_helpers',A.parent/'current_input_jetson_r296_v1/measure_cost.py')
candidate=module('r351_saved_policy',A.parent/'current_solo_gap_r346_v1/candidate.py')


def main():
    assert not (A/'report.json').exists()
    source=A.parent/'final_cad_solo_gap_drive_r349_v1/run1/follow_decisions.jsonl'
    with source.open() as stream:
        header=json.loads(next(stream))['data']
        calls=[e['data'] for e in map(json.loads,stream) if e['kind']=='call' and 'current_rebase' in e['data']]
    relevant=[i for i,c in enumerate(calls) if c['current_rebase']['output']['reason']=='route_current_large_motion_masked_toward_right']
    chosen=sorted({relevant[0],relevant[len(relevant)//2],relevant[-1]})
    old.configure('native_v1')
    motion=old.MotionConfig(**header['motion_config']);cfg=old.FollowConfig(**header['follow_config'])
    rows=[]
    for i in chosen:
        call=calls[i];current=call['current_rebase']['input']
        d,scan,history=old.restore_call(current,motion)
        scene=old.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
            epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        submitted=old.plan_reference(call['input'])
        replay=old.freeze(candidate.rebase(call['output'],submitted,current,scene,history,cfg,require_current_region=True))
        assert old.equal_output(replay,call['current_rebase']['output']),(i,'original full output differs')
        trials=[]
        for cap in [call['output'][0],.15,.10,.08,.05]:
            low=(cap,call['output'][1],call['output'][2])
            result=candidate.prior.base.rebase(low,submitted,current,scene,history,cfg,require_current_region=True)
            trials.append(dict(cap_mps=cap,result_mps=result['speed_mps'],reason=result['reason'],
                region=result.get('current_route_proof',{}).get('region'),reference_s=result['reference_s'],
                steering_rad=result['steering_rad']))
        rows.append(dict(index=i,reference_s=d['reference_s'],measured_speed_mps=d['speed_mps'],
            original_full_equal=True,trials=trials))
    report=dict(scope='saved current inputs; no new motion or threshold changes',rows=rows,
        failed_calls=len(relevant),all_current_calls=len(calls),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        default_changed=False,unknown_filled=False,real_vehicle_proof=False)
    (A/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(rows,ensure_ascii=False))


if __name__=='__main__':main()
