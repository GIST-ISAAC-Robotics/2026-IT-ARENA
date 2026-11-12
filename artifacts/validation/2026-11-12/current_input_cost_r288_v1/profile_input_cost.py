"""현재 정책의 입력 종류별 PC 비용 대조. Jetson 추정/실제 주행이 아니다."""
import cProfile
import hashlib
import importlib.util
import json
from pathlib import Path
import pstats
import statistics
import sys
import time
from collections import Counter

A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.follow_trace import restore_call,freeze,equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.numeric_kernels import configure
from overlap import plan_reference
CANDIDATE_PATH=A.parent/'current_region_policy_r281_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r281_profile_candidate',CANDIDATE_PATH)
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)


def source(name):
    path=A.parent/name/'two_actor_run1/run1/follow_decisions.jsonl'
    header=None;calls=[]
    with path.open() as f:
        for line in f:
            e=json.loads(line)
            if e['kind']=='header':header=e['data']
            if e['kind']=='call' and e['data'].get('current_rebase'):
                calls.append(e['data'])
    return path,header,calls


def main():
    configure('native_v1')
    sources={}
    for label,name in [('r279','opposite_reacquire_runtime_r279_v1'),('r282','current_region_runtime_r282_v1')]:
        path,h,calls=source(name)
        sources[label]=dict(path=path,header=h,calls=calls,cfg=FollowConfig(**h['follow_config']),motion=MotionConfig(**h['motion_config']))
    rows=[];stored=[];profile=cProfile.Profile()
    def evaluate(s,c):
        current=c['current_rebase']['input'];d,scan,h=restore_call(current,s['motion'])
        scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        return freeze(candidate.rebase(c['output'],plan_reference(c['input']),current,scene,h,s['cfg'],require_current_region=True))
    # 각 원 입력의 매10번째를 순서 그대로 대조하고 그룹 순서를 교대한다.
    for rank in range(max(len(s['calls']) for s in sources.values())//10+1):
        labels=['r279','r282'] if rank%2==0 else ['r282','r279']
        for label in labels:
            s=sources[label];index=rank*10
            if index>=len(s['calls']):continue
            c=s['calls'][index]
            started=time.perf_counter_ns();out=evaluate(s,c);elapsed=(time.perf_counter_ns()-started)/1e6
            if label=='r282':assert equal_output(out,c['current_rebase']['output']),index
            rows.append(dict(source=label,index=index,elapsed_ms=elapsed,reason=out['reason'],speed_mps=out['speed_mps']))
            if rank%5==0:
                instrumented=profile.runcall(evaluate,s,c);assert equal_output(out,instrumented)
            stored.append(dict(source=label,index=index,input=c,output=out))
    def stats(values):
        q=sorted(values)
        return dict(n=len(q),median=statistics.median(q),p95=q[int((len(q)-1)*.95)],maximum=q[-1])
    report=dict(groups={label:dict(cost_ms=stats([r['elapsed_ms'] for r in rows if r['source']==label]),
                     reasons=dict(Counter(r['reason'] for r in rows if r['source']==label))) for label in sources},
                source_sha256={label:hashlib.sha256(s['path'].read_bytes()).hexdigest() for label,s in sources.items()},
                candidate_sha256=hashlib.sha256(CANDIDATE_PATH.read_bytes()).hexdigest(),
                scope='PC every tenth recorded current input, alternating source order; no IPC/concurrent sensors/ROS/Jetson conversion; cProfile calls excluded from cost rows')
    stat=pstats.Stats(profile)
    top=[]
    for (file,line,name),(primitive,total,self_s,cumulative_s,_) in sorted(stat.stats.items(),key=lambda x:x[1][3],reverse=True)[:25]:
        top.append(dict(file=file,line=line,name=name,primitive=primitive,calls=total,self_s=self_s,cumulative_s=cumulative_s))
    (A/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (A/'durations.json').write_text(json.dumps(rows,indent=2)+'\n')
    (A/'top_profile.json').write_text(json.dumps(top,indent=2)+'\n')
    (A/'selected_inputs.json').write_text(json.dumps(stored,separators=(',',':'))+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
