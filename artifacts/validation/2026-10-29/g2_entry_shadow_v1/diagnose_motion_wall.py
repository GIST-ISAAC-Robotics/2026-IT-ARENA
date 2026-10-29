"""저장 적용 운동으로 센서 생산자를 재구성하고 원래 판단부터 대조한다."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryConfig,EntryGuide,observed_straight_wall
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan
from bypass_probe_sensors import ProbeSensors

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]

def inspect_state(guide,scene,decision):
    detail={'reason':decision['reason'],'reference':decision.get('reference'),
      'disagreement_m':decision.get('disagreement_m'),'phase':guide.phase}
    if guide.path is None or guide.wall_line is None:return detail
    fit,wall=observed_straight_wall(scene,guide.config,guide.side)
    detail.update(wall=wall,side=guide.side,path_end=guide.path[-1].tolist(),offset=guide.offset)
    if fit is None:return detail
    slope,intercept,lo,hi=fit
    old=guide.wall_line
    old_slope=(old[1,1]-old[0,1])/(old[1,0]-old[0,0])
    old_b=old[0,1]-old_slope*old[0,0]
    detail.update(current_heading_rad=math.atan(slope),predicted_heading_rad=math.atan(old_slope),
      wall_rotation_rad=math.atan(slope)-math.atan(old_slope),
      wall_normal_shift_m=intercept/math.sqrt(1+slope*slope)-old_b/math.sqrt(1+old_slope*old_slope),
      extension_error_m=float(guide.path[-1,1]-(slope*guide.path[-1,0]+intercept+guide.offset*math.sqrt(1+slope*slope))))
    return detail

def replay(folder):
    source=json.loads((folder/'report.json').read_text())
    # 동일 제어 소스만 재구성한다. 해시 불일치 때 과거 결과를 대체하지 않는다.
    changed=[name for name,digest in source['source_sha256'].items() if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
    if changed:raise ValueError('source differs: '+str(changed))
    cases=[]
    for filename in sorted(folder.glob('y*.json')):
        raw=filename.read_bytes();saved=json.loads(raw)
        cfg=FollowConfig();entry=EntryConfig(offsets_m=(0.,-.16,.16,-.20,.20,-.24,.24,-.27,.27))
        guide=EntryGuide(cfg,entry,stationary_outer_boundary=True,rejoin_enabled=True)
        history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=5.))
        sensors=ProbeSensors(saved['sensor_profile']['profile'],saved['seed']);physical=[]
        for k in range(25):
            at=k*.02;physical.append(dict(t=at,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.))
            mean,gyro,count=sensors.measure_motion(at,0.,0.,0.)
            history.add_mean_wheel(at,mean);history.add_gyro(at,gyro)
        previous=travel=0.;matched=0;seen=set();diagnostics=[]
        for k,r in enumerate(saved['records']):
            now=r['t'];cx,cy,yaw=r['evaluation']['center_pose']
            rear=[cx-.0675*math.cos(yaw),cy-.0675*math.sin(yaw),yaw]
            physical.append(dict(t=now,rear_pose=rear,speed=r['actual_speed_mps'],steering=previous))
            if sensors.motion_enabled:
                mean,gyro,count=sensors.measure_motion(now,travel,r['actual_speed_mps'],previous)
                history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
            if k%5==0:scan=render_scan(physical,now,saved['target_y'],saved['target_restarts'],sensors)
            scene=motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=30.)
            _,values=history.wheels.available(now);speed=float(values[-1])
            decision=guide.evaluate(scene,history,speed,previous,.02)
            chosen,expected=decision['selected'],r['selected']
            same=(chosen is None)==(expected is None) and decision['reason']==r['reason'] and decision.get('entry_phase')==r['entry_phase']
            if chosen and expected:same=same and all(abs(chosen[key]-expected[key])<1e-9 for key in ('speed_mps','steering_rad','offset_m'))
            if not same:raise AssertionError(f'{folder.name}/{filename.name} at {now}: judgment mismatch')
            matched+=1
            signature=(r['reason'],tuple(sorted(r['candidate_reasons'])))
            if r['phase']=='entry' and (signature not in seen or k==len(saved['records'])-32):
                seen.add(signature)
                diagnostics.append(dict(t=now,measured_speed=speed,actual_speed=r['actual_speed_mps'],
                  state=inspect_state(guide,scene,decision),candidates=r['candidate_summary']))
            next_speed=float(np.clip(r['safe'][0],max(0.,r['actual_speed_mps']-.04),r['actual_speed_mps']+.02))
            previous=float(r['safe'][1]);physical[-1].update(speed=next_speed,steering=previous)
            if not sensors.motion_enabled:
                history.add_mean_wheel(now,next_speed/.0325);history.add_gyro(now,next_speed*math.tan(previous)/.135)
            travel+=next_speed*.02
        cases.append(dict(file=filename.name,input_sha256=hashlib.sha256(raw).hexdigest(),matched=matched,diagnostics=diagnostics))
    return dict(folder=folder.name,source_matched=True,cases=cases)

def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);args=p.parse_args()
    out=BASE/(args.folder.name+'_wall_diagnosis.json')
    if out.exists():raise ValueError('preserve prior result')
    data=replay(args.folder)
    out.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps([{'file':c['file'],'matched':c['matched'],'diagnostics':len(c['diagnostics'])} for c in data['cases']]))
if __name__=='__main__':main()
