"""r288의 고정 대표 입력을 실제 플랫폼에서 재측정. 센서·IPC·ROS 부하 제외."""
from pathlib import Path
import argparse, hashlib, json, platform, statistics, sys, time
A=Path(__file__).resolve().parent;R=A.parents[3];P=R/'artifacts/validation/2026-11-12'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(P/'retained_route_execution_r266_v1'),str(P/'current_region_policy_r281_v1')]
from arena_autonomy.follow_trace import restore_call,freeze,equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.numeric_kernels import configure
from overlap import plan_reference
import candidate

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    args.out.mkdir(exist_ok=False)
    manifest=json.loads((A/'source_manifest.json').read_bytes())
    def changed():return [p for p,h in manifest.items() if hashlib.sha256((R/p).read_bytes()).hexdigest()!=h]
    assert not changed()
    configure('native_v1')
    headers=json.loads((A/'headers.json').read_bytes());rows=[];answers=[]
    configs={k:(MotionConfig(**v['motion_config']),FollowConfig(**v['follow_config'])) for k,v in headers.items()}
    fields=('speed_mps','steering_rad','reason','reference_s','original_reference_s','source_first_s','path_source_s','independent_guard_required','runtime_authorized')
    for entry in json.loads((A/'selected_inputs.json').read_bytes()):
        c=entry['input'];motion,cfg=configs[entry['source']]
        start=time.perf_counter_ns()
        current=c['current_rebase']['input'];d,scan,history=restore_call(current,motion)
        scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        out=freeze(candidate.rebase(c['output'],plan_reference(c['input']),current,scene,history,cfg,require_current_region=True))
        elapsed=(time.perf_counter_ns()-start)/1e6
        expected=entry['output']
        rows.append(dict(source=entry['source'],index=entry['index'],elapsed_ms=elapsed,
            full_equal=equal_output(out,expected),operational_equal=equal_output({k:out.get(k) for k in fields},{k:expected.get(k) for k in fields})))
        answers.append(out)
    def stats(values):
        v=sorted(values);return dict(n=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],maximum=v[-1])
    groups={label:dict(cost_ms=stats([r['elapsed_ms'] for r in rows if r['source']==label]),full_equal=sum(r['full_equal'] for r in rows if r['source']==label),operational_equal=sum(r['operational_equal'] for r in rows if r['source']==label)) for label in headers}
    report=dict(platform=platform.platform(),python=sys.version,groups=groups,changed_sources=changed(),scope='fixed r288 every-tenth current input; no sensors/IPC/ROS/concurrent planner; not a worst-case latency bound')
    for name,value in [('report',report),('durations',rows),('answers',answers)]:
        (args.out/(name+'.json')).write_text(json.dumps(value,separators=(',',':'))+'\n')
    print(json.dumps(report));assert not report['changed_sources']
    assert all(r['operational_equal'] for r in rows),'operational output changed'

if __name__=='__main__':main()
