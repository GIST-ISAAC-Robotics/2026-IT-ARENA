"""원 r364 전체 현재 입력과 과거 교통106개에서 명시 후보를 비교한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(A))
import candidate
from arena_autonomy.follow_trace import restore_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.runtime_dual_process import plan_reference
from arena_autonomy.numeric_kernels import configure


def main():
    configure('native_v1')
    trace=A.parent/'released_crawl_drive_r364_v1/run1/follow_decisions.jsonl'
    events=[json.loads(x) for x in trace.open()]
    h=events[0]['data']
    records=[]
    for e in events:
        if e['kind']!='call' or 'current_rebase' not in e['data']:continue
        row=e['data'];c=row['current_rebase'];plan=c['compute']['plan']
        records.append(('r364',h,plan['output'],plan_reference(plan['input']),c['input']))
    hold=A.parent/'current_input_jetson_r296_v1'
    headers=json.loads((hold/'headers.json').read_bytes())
    for e in json.loads((hold/'selected_inputs.json').read_bytes()):
        row=e['input']
        records.append(('holdout_'+e['source'],headers[e['source']],row['output'],
                        plan_reference(row['input']),row['current_rebase']['input']))
    rows=[];configs={};answers=[]
    for index,(label,h,out,submitted,current) in enumerate(records):
        if label not in configs:configs[label]=(MotionConfig(**h['motion_config']),FollowConfig(**h['follow_config']))
        motion,cfg=configs[label]
        info,scan,history=restore_call(current,motion)
        scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
                           epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        result={};cost={}
        order=[('before',candidate.prior),('after',candidate)]
        if index%2:order.reverse()
        for mode,mod in order:
            start=time.perf_counter_ns()
            result[mode]=freeze(mod.rebase(out,submitted,current,scene,history,cfg,require_current_region=True))
            cost[mode]=(time.perf_counter_ns()-start)/1e6
        before,after=result['before'],result['after']
        retry=after.get('current_relative_body_retry') or {}
        applied=retry.get('applied') is True
        if applied:
            assert before['speed_mps']==0 and 0<after['speed_mps']<=.1
            assert before['steering_rad']==after['steering_rad'] and before['reference_s']==after['reference_s']
            assert after['current_route_proof']['accepted'] and not retry['unknown_filled']
        else:
            assert equal_output(before,{k:v for k,v in after.items() if k!='current_relative_body_retry'})
        rows.append(dict(index=index,label=label,reference_s=current['reference_s'],applied=applied,
                         before_speed=before['speed_mps'],after_speed=after['speed_mps'],cost_ms=cost,
                         diagnostic=retry))
        answers.append(result)
    hashes={}
    for mod in list(sys.modules.values()):
        name=getattr(mod,'__file__',None)
        if not name:continue
        path=Path(name).resolve()
        if path.is_file() and path.is_relative_to(ROOT) and path.suffix in ('.py','.so'):
            hashes[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in hashes:z.write(ROOT/name,name)
    groups={label:dict(n=sum(r['label']==label for r in rows),
                      applied=sum(r['applied'] for r in rows if r['label']==label)) for label in configs}
    report=dict(groups=groups,rows=rows,source_hashes=hashes,trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        scope='stored inputs only; optional current policy; not a drive or hardware proof')
    (A/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    (A/'answers.json').write_text(json.dumps(answers)+'\n')
    print(json.dumps(groups))


if __name__=='__main__':main()
