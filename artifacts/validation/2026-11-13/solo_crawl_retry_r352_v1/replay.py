import hashlib
import importlib.util
import json
from pathlib import Path
import time
import numpy as np
import candidate

A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r352_helpers',A.parent/'current_input_jetson_r296_v1/measure_cost.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)


def main():
    assert not (A/'report.json').exists()
    source=A.parent/'final_cad_solo_gap_drive_r349_v1/run1/follow_decisions.jsonl'
    with source.open() as stream:
        header=json.loads(next(stream))['data']
        rows=[('r349',header,r['data']) for r in map(json.loads,stream) if r['kind']=='call' and 'current_rebase' in r['data']]
    hold=A.parent/'current_input_jetson_r296_v1'
    headers=json.loads((hold/'headers.json').read_bytes())
    rows.extend((r['source'],headers[r['source']],r['input']) for r in json.loads((hold/'selected_inputs.json').read_bytes()))
    old.configure('native_v1');configs={};results=[]
    for i,(label,h,call) in enumerate(rows):
        if label not in configs:configs[label]=(old.MotionConfig(**h['motion_config']),old.FollowConfig(**h['follow_config']))
        motion,cfg=configs[label];current=call['current_rebase']['input']
        d,scan,history=old.restore_call(current,motion)
        scene=old.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
            epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        answer={};cost={};order=[('before',candidate.prior),('after',candidate)]
        if i%2:order.reverse()
        for name,module in order:
            start=time.perf_counter_ns()
            answer[name]=old.freeze(module.rebase(call['output'],old.plan_reference(call['input']),current,
                                                 scene,history,cfg,require_current_region=True))
            cost[name]=(time.perf_counter_ns()-start)/1e6
        before,after=answer['before'],answer['after']
        if label=='r349':
            expected=call['current_rebase']['output'];cap=expected.get('new_plan_deceleration_cap')
            replay=before
            if cap is not None:
                assert old.equal_output(before['speed_mps'],cap['checked_target_mps'])
                assert cap['steering_changed'] is False
                assert cap['current_check_reused_without_timestamp_renewal'] is True
                assert cap['applied'] == (min(before['speed_mps'],cap['speed_mps']) < before['speed_mps'])
                replay={**before,'speed_mps':min(before['speed_mps'],cap['speed_mps']),
                        'new_plan_deceleration_cap':cap}
            assert old.equal_output(replay,expected),(i,'original full output differs')
        retry=after.get('current_solo_crawl_retry',{})
        assert before['steering_rad']==after['steering_rad']
        assert before['reference_s']==after['reference_s']
        if after['speed_mps']>before['speed_mps']:
            assert before['speed_mps']==0 and 0<after['speed_mps']<=.1 and retry['applied'] is True
            assert after['current_route_proof']['accepted'] is True
        else:
            assert old.equal_output({k:v for k,v in after.items() if k!='current_solo_crawl_retry'},before)
        results.append(dict(index=i,source=label,reference_s=d['reference_s'],changed=not old.equal_output(before,after),
            before_mps=before['speed_mps'],after_mps=after['speed_mps'],reason=after['reason'],
            applied=retry.get('applied'),cost_ms=cost,region=after.get('current_route_proof',{}).get('region')))
    summary={}
    for label in configs:
        selected=[r for r in results if r['source']==label]
        summary[label]=dict(calls=len(selected),changed=sum(r['changed'] for r in selected),
            applied=sum(r['applied'] is True for r in selected),positive_changes=sum(r['after_mps']>r['before_mps'] for r in selected),
            cost_ms={k:dict(p50=float(np.median([r['cost_ms'][k] for r in selected])),
                           p95=float(np.percentile([r['cost_ms'][k] for r in selected],95))) for k in ('before','after')})
    (A/'report.json').write_text(json.dumps(dict(summary=summary,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        scope='same recorded inputs, old 135mm geometry; no drive or Jetson timing proof'),ensure_ascii=False,indent=2)+'\n')
    (A/'answers.json').write_text(json.dumps(results,separators=(',',':'))+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
