"""같은 호출의 영역 중복 생략 전후. 입력 복원부터 반환까지 같은 범위다."""
import argparse,hashlib,json,statistics,sys,time
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.follow_trace import restore_call,equal_output,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.numeric_kernels import configure
import candidate,candidate_before_region_reuse as before


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    args.out.mkdir(exist_ok=False)
    data=json.loads((A/'profile_inputs.json').read_bytes())
    cfg=FollowConfig(**data['header']['follow_config']);motion=MotionConfig(**data['header']['motion_config']);configure('native_v1')
    rows=[];answers=[]
    for i,c in enumerate(data['calls']):
        order=[('before',before),('after',candidate)]
        if i%2:order.reverse()
        outputs={};cost={}
        for label,module in order:
            start=time.perf_counter_ns()
            current=c['current'];d,scan,h=restore_call(current,motion)
            scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
            outputs[label]=freeze(module.rebase(c['output'],c['submitted'],current,scene,h,cfg,require_current_region=True))
            cost[label]=(time.perf_counter_ns()-start)/1e6
        assert equal_output(outputs['before'],outputs['after']),i
        rows.append(cost);answers.append(outputs['after'])
    def stats(k):
        v=sorted(x[k] for x in rows)
        return dict(median=statistics.median(v),p95=v[int((len(v)-1)*.95)],maximum=max(v))
    report=dict(calls=len(rows),all_output_equal=True,full_ms={k:stats(k) for k in ('before','after')},
        inputs_sha256=hashlib.sha256((A/'profile_inputs.json').read_bytes()).hexdigest(),
        candidate_sha256=hashlib.sha256((A/'candidate.py').read_bytes()).hexdigest(),
        scope='single-process input restore + current scene + recheck; no IPC, sensors, ROS or concurrent planner load')
    (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.out/'answers.json').write_text(json.dumps(answers,separators=(',',':'))+'\n')
    (args.out/'durations.json').write_text(json.dumps(rows,separators=(',',':'))+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
