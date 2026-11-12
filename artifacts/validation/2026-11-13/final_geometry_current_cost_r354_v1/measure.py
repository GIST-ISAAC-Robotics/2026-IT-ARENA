"""현재 확정 기하 저장 입력의 계산 비용. 실제 I/O/ROS 병렬 부하와 구분한다."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import platform
import statistics
import sys
import time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
helper=load('r354_helper',A.parent/'current_input_jetson_r296_v1/measure_cost.py')
policy=load('r354_policy',A.parent/'solo_crawl_retry_r352_v1/candidate.py')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(exist_ok=False)
    manifest=json.loads((A/'source_manifest.json').read_bytes())
    def changed():return [n for n,h in manifest.items() if not (ROOT/n).is_file() or hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    assert not changed(),'source drift before measurement'
    data=json.loads((A/'selected.json').read_bytes());h=data['header']
    motion,cfg=helper.MotionConfig(**h['motion_config']),helper.FollowConfig(**h['follow_config'])
    assert cfg.wheelbase_m==.128 and cfg.length_m==.207 and cfg.width_m==.15
    helper.configure('native_v1');records=[];outputs=[]
    for item in data['rows']:
        c=item['call'];current=c['current_rebase']['input'];expected=c['current_rebase']['output']
        t0=time.perf_counter_ns()
        d,scan,history=helper.restore_call(current,motion)
        scene=helper.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
            epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        t1=time.perf_counter_ns()
        answer=helper.freeze(policy.rebase(c['output'],helper.plan_reference(c['input']),current,scene,history,cfg,require_current_region=True))
        t2=time.perf_counter_ns()
        # 발행 경계의 동시 새 계획 감속 상한은 당시 원문과 검사 대상 일치를 확인한 뒤 적용한다.
        cap=expected.get('new_plan_deceleration_cap')
        if cap is not None:
            assert helper.equal_output(answer['speed_mps'],cap['checked_target_mps'])
            assert cap['steering_changed'] is False and cap['current_check_reused_without_timestamp_renewal'] is True
            assert cap['applied']==(min(answer['speed_mps'],cap['speed_mps'])<answer['speed_mps'])
            answer={**answer,'speed_mps':min(answer['speed_mps'],cap['speed_mps']),'new_plan_deceleration_cap':cap}
        records.append(dict(index=item['index'],reference_s=current['reference_s'],reason=answer['reason'],
            full_equal=helper.equal_output(answer,expected),restore_ms=(t1-t0)/1e6,current_ms=(t2-t1)/1e6,total_ms=(t2-t0)/1e6))
        outputs.append(answer)
    def stats(values):
        v=sorted(values);return dict(n=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],maximum=v[-1])
    result=dict(platform=platform.platform(),python=sys.version,geometry=dict(length_m=cfg.length_m,width_m=cfg.width_m,wheelbase_m=cfg.wheelbase_m),
        calls=len(records),full_equal=sum(r['full_equal'] for r in records),cost={key:stats([r[key] for r in records]) for key in ('restore_ms','current_ms','total_ms')},
        changed_sources=changed(),scope='single current-check process, recorded final geometry; no sensors/ROS/planner contention or worst-case guarantee')
    for name,value in [('report',result),('durations',records),('answers',outputs)]:
        (args.output/(name+'.json')).write_text(json.dumps(value,separators=(',',':'))+'\n')
    print(json.dumps(result));assert not result['changed_sources'] and result['full_equal']==len(records)

if __name__=='__main__':main()
