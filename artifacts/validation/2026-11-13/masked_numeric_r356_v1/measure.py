"""동일 입력/플랫폼에서 가림 분류만 NumPy와 native로 교대 비교한다."""
from pathlib import Path
import hashlib
import importlib.util
import json
import platform
import statistics
import sys
import time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
def load(name,path):
    s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
helper=load('r356_helper',A.parent/'current_input_jetson_r296_v1/measure_cost.py')
policy=load('r356_policy',A.parent/'solo_crawl_retry_r352_v1/candidate.py')
from arena_autonomy import numeric_kernels

def main():
    out=Path(sys.argv[1]);out.mkdir(exist_ok=False)
    manifest=json.loads((A/'source_manifest.json').read_bytes())
    def changed():return [n for n,h in manifest.items() if not (ROOT/n).is_file() or hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    assert not changed()
    payload=json.loads((A/'inputs.json').read_bytes());header=payload['header']
    motion=helper.MotionConfig(**header['motion_config']);cfg=helper.FollowConfig(**header['follow_config'])
    helper.configure('native_v1');original_masked=numeric_kernels.masked
    records=[];answers=[]
    for i,call in enumerate(payload['calls']):
        current=call['current_rebase']['input'];expected=call['current_rebase']['output']
        d,scan,history=helper.restore_call(current,motion)
        scene=helper.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        values={};cost={}
        for mode in (['numpy','native'] if i%2==0 else ['native','numpy']):
            numeric_kernels.masked=original_masked if mode=='native' else lambda *a,**k:None
            start=time.perf_counter_ns()
            result=helper.freeze(policy.rebase(call['output'],helper.plan_reference(call['input']),current,scene,history,cfg,require_current_region=True))
            cost[mode]=(time.perf_counter_ns()-start)/1e6
            cap=expected.get('new_plan_deceleration_cap')
            if cap:
                assert helper.equal_output(result['speed_mps'],cap['checked_target_mps'])
                assert cap['steering_changed'] is False and cap['current_check_reused_without_timestamp_renewal'] is True
                result={**result,'speed_mps':min(result['speed_mps'],cap['speed_mps']),'new_plan_deceleration_cap':cap}
            values[mode]=result
        records.append(dict(index=i,reference_s=current['reference_s'],reason=values['native']['reason'],cost_ms=cost,
            paired_full_equal=helper.equal_output(values['numpy'],values['native']),
            recorded_full_equal={k:helper.equal_output(v,expected) for k,v in values.items()}))
        answers.append(values)
    numeric_kernels.masked=original_masked
    def stats(v):
        v=sorted(v);return dict(n=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],maximum=v[-1])
    report=dict(platform=platform.platform(),calls=len(records),paired_full_equal=sum(r['paired_full_equal'] for r in records),
        recorded_full_equal={mode:sum(r['recorded_full_equal'][mode] for r in records) for mode in ('numpy','native')},
        current_cost_ms={mode:stats([r['cost_ms'][mode] for r in records]) for mode in ('numpy','native')},
        changed_sources=changed(),scope='same current-input compute only; no planner/ROS/sensor contention or drive proof')
    for name,value in [('report',report),('durations',records),('answers',answers)]:
        (out/(name+'.json')).write_text(json.dumps(value,separators=(',',':'))+'\n')
    print(json.dumps(report));assert not report['changed_sources'] and report['paired_full_equal']==len(records)

if __name__=='__main__':main()
