"""prototype2의 저장 적용 운동을 재구성하고 명령 원호의 첫 간섭을 분리한다."""
import hashlib
import json
import math
import numpy as np
from analyze import BASE, ROOT
import run_prototype as trial
from prototype import choose_initial_intent
from arena_autonomy.bypass_entry_trial import EntryGuide, EntryConfig, current_outer_wall_padding
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import freeze
from bypass_probe_sensors import ProbeSensors
from probe_bypass_entry_sequence import render_scan


def collisions(scene, guide, result, previous):
    row = result['candidates'][0]
    points = np.asarray(scene.occupied_endpoints_xy_m)
    padding = per_ray_padding(scene,.055,.5)
    classification = None
    if guide.wall_confirmations >= 2:
        wall = result['reference']['walls'][guide.side]
        padding, classification = current_outer_wall_padding(scene,guide.config,guide.side,wall)
    rays = [r for r in scene.rays if r.status=='return']
    rows=[]
    for steering in sorted({previous,row['steering_rad']}):
        curve=math.tan(steering)/.135
        for distance in np.arange(0.,2.501,.04):
            yaw=curve*distance
            if abs(curve)<1e-5:
                rx,ry=distance-.0675,0.
            else:
                rx,ry=math.sin(yaw)/curve-.0675,(1-math.cos(yaw))/curve
            dx,dy=points[:,0]-rx,points[:,1]-ry
            x=math.cos(yaw)*dx+math.sin(yaw)*dy-.0675
            y=-math.sin(yaw)*dx+math.cos(yaw)*dy
            hit=np.flatnonzero((abs(x)<=.1+padding)&(abs(y)<=.075+padding))
            if len(hit):
                rows.append(dict(steering=steering,first_sample_distance_m=float(distance),
                    clearance_m=max(0.,float(distance)-.04),
                    returns=[dict(point=points[i].tolist(),padding_m=float(padding[i]),
                        acquired_at_s=rays[i].acquired_at_s,age_s=scene.t_ref_s-rays[i].acquired_at_s,
                        source_index=rays[i].source_index,
                        swept_local_xy=[float(x[i]),float(y[i])]) for i in hit]))
                break
    return dict(first_interference=rows,classification=classification,
                points=points.tolist(),padding_m=padding.tolist(),candidate=row,
                source_first_s=scene.source_first_s,reference_s=scene.t_ref_s,
                outer_wall_confirmations=guide.wall_confirmations)


def main():
    folder=BASE/'prototype2'
    report=json.loads((folder/'report.json').read_text())
    changed=[n for n,h in report['source_sha256'].items() if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    if changed:
        raise RuntimeError('captured source changed: '+str(changed))
    raw=folder/'y0.0_restart0.json'
    saved=json.loads(raw.read_text())
    guide=EntryGuide(FollowConfig(),EntryConfig(offsets_m=(0.,-.16,.16,-.20,.20,-.24,.24,-.27,.27)),
                     stationary_outer_boundary=True,rejoin_enabled=True,wall_reanchor=True,tracked_wall_fit=True)
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=5.))
    sensors=ProbeSensors('combined',17)
    physical=[]
    for k in range(25):
        now=k*.02
        physical.append(dict(t=now,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.))
        mean,gyro,_=sensors.measure_motion(now,0.,0.,0.)
        history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
    travel=previous=0.
    matched=0;selected=[];first_failure=False
    for k,r in enumerate(saved['records']):
        now=r['t'];cx,cy,yaw=r['evaluation']['center_pose']
        rear=[cx-.0675*math.cos(yaw),cy-.0675*math.sin(yaw),yaw]
        physical.append(dict(t=now,rear_pose=rear,speed=r['actual_speed_mps'],steering=previous))
        mean,gyro,_=sensors.measure_motion(now,travel,r['actual_speed_mps'],previous)
        history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
        if k%5==0:scan=render_scan(physical,now,0.,False,sensors)
        scene=motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=30.)
        speed=float(history.wheels.available(now)[1][-1])
        result=guide.evaluate(scene,history,speed,previous,.02)
        got,expected=result['selected'],r['selected']
        same=result['reason']==r['reason'] and (got is None)==(expected is None)
        if got and expected:
            same=same and all(abs(got[name]-expected[name])<1e-9 for name in ('speed_mps','steering_rad','offset_m','transition_m'))
        if not same:
            raise AssertionError(f'judgment mismatch at {now}')
        matched+=1
        first=not first_failure and result['reason']=='no_entry_command_candidate'
        if first or now in (3.2,3.28,9.5,9.58):
            first_failure |= first
            selected.append(dict(t=now,reason=result['reason'],selected=got,safe=r['safe'],
                truth_pose=r['evaluation']['center_pose'],measured_speed_mps=speed,
                geometry=collisions(scene,guide,result,previous)))
        next_speed=float(np.clip(r['safe'][0],max(0.,r['actual_speed_mps']-.04),r['actual_speed_mps']+.02))
        previous=float(r['safe'][1])
        physical[-1].update(speed=next_speed,steering=previous)
        travel+=next_speed*.02
    out=BASE/'prototype2_stop_diagnosis.json'
    if out.exists():raise RuntimeError('preserve prior result')
    out.write_text(json.dumps(freeze(dict(matched=matched,source_matched=True,input_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
                    scope='saved input reconstruction; truth pose only used to reproduce original sensor producer',samples=selected)),indent=2)+'\n')
    print(json.dumps([dict(t=r['t'],reason=r['reason'],safe=r['safe'],interference=r['geometry']['first_interference']) for r in selected],indent=2))


if __name__ == '__main__':
    trial.entry_module._compare_entries=trial.comparison
    trial.entry_module.EntryIntent.select=lambda self,scene,result:choose_initial_intent(self,scene,result,trial.original_select)
    try:main()
    finally:
        trial.entry_module._compare_entries=trial.original
        trial.entry_module.EntryIntent.select=trial.original_select
