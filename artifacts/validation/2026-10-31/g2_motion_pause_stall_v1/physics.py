"""평가 전용 실제 차체 운동과 모터/바퀴의 차이를 회수한다. 제어기 입력 아님."""
import hashlib,json
from pathlib import Path
BASE=Path(__file__).resolve().parent
RUN=BASE.parent/'g2_motion_pause_gazebo_v1/run1'


def sha(p):
    with p.open('rb') as s:return hashlib.file_digest(s,'sha256').hexdigest()


def main():
    files=[RUN/n for n in ('braking_diagnostics.jsonl','trajectory.jsonl','report.json')]
    hashes={p.name:sha(p) for p in files}
    rows=[json.loads(s) for s in files[0].open()]
    keys=('sim_time_s','applied_target_mps','left_speed_rad_s','right_speed_rad_s','wheel_surface_speed_mps',
        'truth_longitudinal_mps','truth_roll_rad','truth_pitch_rad','truth_yaw_rad','truth_x_m','truth_y_m',
        'motor_torque_nm','raw_demand_nm','pi_integral_after_nm','left_steering_rad','right_steering_rad')
    samples=[]
    for t in (23.,24.,24.6,25.,26.,27.,28.,28.5,29.,29.8,30.2,31.,32.,40.,154.6,154.72):
        r=min(rows,key=lambda r:abs(r['sim_time_s']-t));samples.append({k:r.get(k) for k in keys})
    intervals=[]
    for lo,hi in ((23.,29.),(29.,32.),(32.,154.6)):
        selected=[r for r in rows if lo<=r['sim_time_s']<=hi]
        wheel=body=0.;maximum_gap=0.
        for a,b in zip(selected,selected[1:]):
            dt=b['sim_time_s']-a['sim_time_s']
            wheel+=.5*(a['wheel_surface_speed_mps']+b['wheel_surface_speed_mps'])*dt
            body+=.5*(a['truth_longitudinal_mps']+b['truth_longitudinal_mps'])*dt
            maximum_gap=max(maximum_gap,dt)
        peak=max(selected,key=lambda r:abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']))
        intervals.append(dict(window_s=[lo,hi],samples=len(selected),max_sample_gap_s=maximum_gap,
            signed_wheel_surface_distance_m=wheel,signed_body_longitudinal_distance_m=body,
            peak_disagreement_sample={k:peak.get(k) for k in keys}))
    trace=[json.loads(s) for s in files[1].open()];before=min(trace,key=lambda r:abs(r['t']-154.6))
    report=json.loads(files[2].read_text())
    result=dict(scope='evaluation truth only; body longitudinal integral is not centerline progress',
        source_sha256=hashes,input_unchanged=all(sha(p)==hashes[p.name] for p in files),
        samples=samples,intervals=intervals,contact=report['evaluation']['gap'] if 'gap' in report['evaluation'] else report['contact_evidence'],
        pre_contact={k:before.get(k) for k in ('t','x','y','yaw','progress_m','truth_longitudinal_mps','drive_speed_mps','safe_speed_mps','actor_x','actor_y','sampled_gap_m')})
    output=BASE/'physics.json'
    with output.open('x',encoding='utf-8') as s:json.dump(result,s,ensure_ascii=False,indent=2)
    print(json.dumps(dict(input_unchanged=result['input_unchanged'],intervals=intervals,pre_contact=result['pre_contact'])))


if __name__=='__main__':main()
