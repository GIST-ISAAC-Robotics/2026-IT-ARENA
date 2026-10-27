"""제동 중 모터축 이동과 평가용 차체 이동 대조. 제어기 입력/실차 증거 아님."""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rows=[json.loads(line) for line in (args.run/'trajectory.jsonl').open(encoding='utf-8')]
    first=next(r for r in rows if ((r.get('safety') or {}).get('partial_observation') or {}).get('latch_evidence'))
    evidence=first['safety']['partial_observation']['latch_evidence']
    begin,end=evidence['previous_reference_s'],evidence['reference_s']
    poses=[json.loads(line) for line in (args.run/'ego_poses.jsonl').open(encoding='utf-8')]
    times=[p[0] for p in poses]
    feedback=json.loads((args.run/'actuation_trace.json').read_text(encoding='utf-8'))
    speeds={r['feedback_capture_us']/1e6:r['mean_wheel_mps'] for r in feedback
            if r.get('feedback_valid') and r.get('feedback_capture_us') is not None
            and r.get('feedback_mode')=='drive_motor_shaft'}
    speed_times=sorted(speeds)

    def pose_at(stamp):
        i=bisect_right(times,stamp)-1
        a,b=poses[i],poses[i+1]
        part=(stamp-a[0])/(b[0]-a[0])
        return (a[1]+part*(b[1]-a[1]),a[2]+part*(b[2]-a[2]),
                a[3]+part*math.remainder(b[3]-a[3],math.tau))

    start,finish=pose_at(begin),pose_at(end)
    rear=lambda p:(p[0]-.0675*math.cos(p[2]),p[1]-.0675*math.sin(p[2]))
    true_start,true_finish=rear(start),rear(finish)
    actual=[b-a for a,b in zip(true_start,true_finish)]
    prediction=[0.,0.]
    count=math.ceil((end-begin)/.001)
    dt=(end-begin)/count
    ages=[]
    for i in range(count):
        stamp=begin+(i+.5)*dt
        j=bisect_right(speed_times,stamp)-1
        capture=speed_times[j]
        ages.append(stamp-capture)
        speed=speeds[capture]
        yaw=pose_at(stamp)[2]
        prediction[0]+=speed*math.cos(yaw)*dt
        prediction[1]+=speed*math.sin(yaw)*dt
    residual=[a-b for a,b in zip(actual,prediction)]
    samples=[]
    for r in rows:
        if begin-.3<=r['time']<=end+.3:
            d=r.get('dynamics',{})
            samples.append(dict(record_time_s=r['time'],dynamics_time_s=d.get('sim_time_s'),
                truth_longitudinal_mps=d.get('truth_longitudinal_mps'),
                truth_lateral_mps=d.get('truth_lateral_mps'),
                wheel_surface_speed_mps=d.get('wheel_surface_speed_mps'),
                reason=r.get('safety',{}).get('reason')))
    result=dict(scope='offline_evaluation_only_ideal_truth_yaw_motor_translation_counterfactual',
        not_exact_controller_replay=True,truth_used_in_controller=False,
        interval_s=[begin,end],rear_axle_x_m=-.0675,
        actual_rear_translation_world_m=actual,motor_translation_with_truth_yaw_world_m=prediction,
        residual_world_m=residual,residual_norm_m=math.hypot(*residual),
        maximum_feedback_age_s=max(ages),samples=samples,
        limits=['ideal truth yaw isolates translation; it is never supplied to the controller',
                'scan deskew error before previous reference is not reconstructed',
                'motor feedback filtering, wheel slip and timestamp effects are not individually identified'],
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       [args.run/'trajectory.jsonl',args.run/'ego_poses.jsonl',args.run/'actuation_trace.json',Path(__file__)]})
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({k:result[k] for k in ['interval_s','residual_norm_m','maximum_feedback_age_s']}))


if __name__=='__main__':main()
