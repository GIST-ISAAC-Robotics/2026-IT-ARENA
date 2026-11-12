"""센서/ROS 없이 현재 입력 복원·영역 검사 전체 범위의 원/후 비용 대조."""
import argparse,hashlib,json,sys,time,statistics
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.follow_trace import restore_call,equal_output,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.numeric_kernels import configure
from overlap import evaluate_current
import candidate


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    data=json.loads((A/'profile_inputs.json').read_bytes());cfg=FollowConfig(**data['header']['follow_config']);motion=MotionConfig(**data['header']['motion_config']);configure('native_v1')
    def after(c):
        current=c['current'];d,scan,h=restore_call(current,motion)
        scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        return freeze(candidate.rebase(c['output'],c['submitted'],current,scene,h,cfg,require_current_region=True))
    def before(c):return evaluate_current(c['output'],c['submitted'],c['current'],motion,cfg)
    answers=[];rows=[];zero_to_positive=0
    for i,c in enumerate(data['calls']):
        order=[('before',before),('after',after)]
        if i%2:order.reverse()
        outputs={};cost={}
        for label,function in order:
            start=time.perf_counter_ns();outputs[label]=function(c);cost[label]=(time.perf_counter_ns()-start)/1e6
        assert equal_output(outputs['before'],c['expected']),i
        zero_to_positive+=outputs['before']['speed_mps']==0 and outputs['after']['speed_mps']>0
        assert not (outputs['before']['speed_mps']>0 and outputs['after']['speed_mps']==0)
        answers.append(outputs['after']);rows.append(cost)
    def stats(k):
        q=sorted(r[k] for r in rows);return dict(median=statistics.median(q),p95=q[int((len(q)-1)*.95)],maximum=max(q))
    report=dict(calls=len(rows),original_equal=len(rows),zero_to_positive=zero_to_positive,full_ms={k:stats(k) for k in ('before','after')},
        inputs_sha256=hashlib.sha256((A/'profile_inputs.json').read_bytes()).hexdigest(),scope='single-process restored current input + check; no IPC, sensors, ROS or concurrent planner load')
    (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.out/'answers.json').write_text(json.dumps(answers,separators=(',',':'))+'\n')
    (args.out/'durations.json').write_text(json.dumps(rows,separators=(',',':'))+'\n');print(json.dumps(report))


if __name__=='__main__':main()
