"""보존된 prototype2 궤적에서 센서를 재구성하여 연결 코드와 비교. 새 주행 아님."""
import hashlib
import argparse
import json
import math
from pathlib import Path
import sys
import zipfile
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe
from bypass_probe_sensors import ProbeSensors
from g0_geometry import TruthBox
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import FollowTrace,freeze,equal_output,comparable_decision,replay_events


def expected_output(row):
    d=dict(row['detail']);state=d.pop('mode')
    d.update(mode='g1_g2_handoff_trial',handoff_state=state,simulation_trial_only=True,
        automatic_race_mode_selection=True,output_requires_independent_guard=True,
        behavior='HOLD' if row['requested'][0]<=0 else state,scope='explicit_simulation_handoff_trial')
    return [*row['requested'],d]


def compare(name,out):
    source=BASE/'prototype2'/(name+'.json')
    old=json.loads(source.read_text())
    ctl=BypassHandoffFollow(allow_simulation_trial=True)
    motion=MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=5.)
    history=MotionHistory(motion);sensors=ProbeSensors(old['sensor_profile']['profile'],17)
    physical=[];travel=0.;previous=0.;pose=(-.0675,0.,0.)
    for k in range(25):
        now=k*.02
        physical.append(dict(t=now,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.))
        mean,gyro,_=sensors.measure_motion(now,0.,0.,0.)
        history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
    original_boxes=probe.evaluation_boxes
    def boxes(at,target_y=0.,restart=False):
        result=original_boxes(at,target_y,False);target=result[-1]
        result[-1]=TruthBox(target.x_m+old['evaluation_target_speed_mps']*max(0.,at-.5),target.y_m,target.height_m)
        return result
    probe.evaluation_boxes=boxes
    trace=FollowTrace(out/(name+'.jsonl'),motion,ctl.config,mode='g1_g2_handoff_trial',execution_config=ctl.execution)
    mismatches=[];calls=0;different=0
    try:
        for k,row in enumerate(old['records']):
            now=row['t'];actual=row['actual_speed_mps'];cx,cy,yaw=row['evaluation']['center_pose']
            # 차체 중심에서 후륜축으로 역환산하면 부동소수점 왕복차가 생긴다.
            # 보존된 실제 적용 명령을 원래 연산 순서로 적분하여 센서 생산자를 복원한다.
            expected_center=[pose[0]+.0675*math.cos(pose[2]),pose[1]+.0675*math.sin(pose[2]),pose[2]]
            if max(abs(a-b) for a,b in zip(expected_center,(cx,cy,yaw)))>1e-12:
                raise ValueError('saved physical trajectory differs from applied-command reconstruction')
            physical.append(dict(t=now,rear_pose=list(pose),speed=actual,steering=previous))
            mean,gyro,_=sensors.measure_motion(now,travel,actual,previous)
            history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
            if k%5==0:raw=probe.render_scan(physical,now,old['target_y'],False,sensors)
            measured=float(history.wheels.available(now)[1][-1])
            if abs(measured-row['measured_speed_mps'])>1e-10:raise ValueError('sensor motion reconstruction differs')
            if row['detail']['mode']=='STOP':
                ctl.reset();trace.emit('reset',{'reason':'saved_scheduled_stop'})
            else:
                scene=motion_scene(raw,history,now,timing_verified=True,rear_half_angle_deg=30.)
                output=ctl.command(scene,history,measured,'left',previous,.02,raw_scan=raw)
                trace.record_call(raw,history,now,0,measured,'left',previous,.02,True,30.,output)
                calls+=1
                if not equal_output(comparable_decision(freeze(output)),comparable_decision(expected_output(row))):
                    different+=1
                    if len(mismatches)<5:mismatches.append(dict(time_s=now,actual=freeze(output),expected=expected_output(row)))
            next_speed=float(np.clip(row['safe'][0],max(0.,actual-2.*.02),actual+1.*.02))
            physical[-1].update(speed=next_speed,steering=row['safe'][1])
            travel+=next_speed*.02;previous=float(row['safe'][1])
            pose=probe.bicycle_step(*pose,next_speed,previous,.02,.135)
    finally:
        probe.evaluation_boxes=original_boxes;trace.close()
    with (out/(name+'.jsonl')).open() as stream:
        replay=replay_events(json.loads(line) for line in stream)
    return dict(name=name,calls=calls,identical_to_prototype2=different==0,mismatch_count=different,mismatches=mismatches,
        recorded_replay={k:v for k,v in replay.items() if k!='outcomes'},
        source_case_sha256=hashlib.sha256(source.read_bytes()).hexdigest())


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',choices=('integration1','integration2','integration3'),default='integration1')
    args=parser.parse_args()
    out=BASE/args.output;out.mkdir(exist_ok=False)
    files=list((ROOT/'src').rglob('*.py'))+list(BASE.glob('*.py'))+list((ROOT/'scripts').glob('*bypass*.py'))
    files+=[ROOT/'scripts/g0_geometry.py',ROOT/'scripts/g2_tracking_probe.py']
    sources={str(p.relative_to(ROOT)):p.read_bytes() for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,raw in sources.items():z.writestr(name,raw)
    result=[]
    for name in ('right_ideal','left_combined','right_combined','moving_combined'):
        item=compare(name,out);result.append(item)
        print(name,item['calls'],item['identical_to_prototype2'],item['recorded_replay']['passed'],flush=True)
    report=dict(cases=result,source_sha256={p:hashlib.sha256(v).hexdigest() for p,v in sources.items()},
        source_unchanged=all((ROOT/p).read_bytes()==v for p,v in sources.items()),scope='stored trajectory sensor reconstruction and decision replay; not new driving')
    (out/'report.json').write_text(json.dumps(freeze(report),indent=2,allow_nan=False)+'\n')
    if not report['source_unchanged'] or any(not r['identical_to_prototype2'] or not r['recorded_replay']['passed'] for r in result):
        raise SystemExit(1)
