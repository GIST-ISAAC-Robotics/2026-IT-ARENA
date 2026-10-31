"""복귀 경로 구성요소의 이상 운동 비교. 전체 센서 제어기/보호층/동역학 시험이 아니다."""
from dataclasses import replace
import importlib.util,hashlib,json,math,zipfile
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryConfig,entry_path,reanchor_path_to_wall
from arena_autonomy.bypass_handoff import handoff_baseline
from arena_autonomy.local_follow import pursue_path
from g2_tracking_probe import bicycle_step

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
source=BASE.parent/'g2_rejoin_refresh_v1/candidate.py'
spec=importlib.util.spec_from_file_location('rejoin_proposal_candidate',source)
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)


def transform(points,dx,dy,yaw):
    rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    return (points-np.array([dx,dy]))@rotation


def fit_from_sensor_generator(rear,wheelbase):
    # 합성 생산자만 실제 자세를 안다. 컨트롤러 부분에는 직선 fit만 전달한다.
    # 실제 광선/가림/벽 연관/잡음 검증을 생략한 이상 현재벽이다.
    x,y,yaw=rear;center_y=y+wheelbase/2*math.sin(yaw)
    return (-math.tan(yaw),-center_y/math.cos(yaw),-.2,2.)


def run_case(enabled,slip):
    cfg=replace(handoff_baseline(),max_steering_rad=.3710583448410034);entry=EntryConfig();dt=.02
    pose=(-cfg.wheelbase_m/2,-.26,0.);fit=fit_from_sensor_generator(pose,cfg.wheelbase_m)
    path=entry_path(fit,0.,cfg,entry);wall=np.array([[fit[2],fit[0]*fit[2]+fit[1]],[fit[3],fit[0]*fit[3]+fit[1]]])
    command_speed=0.;steering=0.;centered=0;first_center=None;refreshes=0;peak=0.;rows=[]
    for k in range(600):
        t=k*dt;fresh=k%5==0;fit=fit_from_sensor_generator(pose,cfg.wheelbase_m)
        correction,_=reanchor_path_to_wall(path,wall,fit)
        if correction is None:return dict(passed=False,error='wall_anchor_outside_existing_budget',time_s=t),rows
        path=correction
        applied=False
        if enabled:
            path,detail=candidate.refresh_rejoin_path(path,fit,cfg,entry,'rejoin',fresh,2,command_speed,.1)
            applied=detail['applied'];refreshes+=applied
        # 현재 가시범위 안에서만 같은 중앙선 끝을 늘린다. 기하 연구이며 자유공간 인증 아님.
        if path[-1,0]<fit[3]:
            xs=np.linspace(path[-1,0],fit[3],max(2,math.ceil((fit[3]-path[-1,0])/.02)+1))
            ys=fit[0]*xs+fit[1]
            path=np.vstack((path,np.c_[xs[1:],ys[1:]]))
        path=path[(path[:,0]>=fit[2])&(path[:,0]<=fit[3])]
        cap,want,_=pursue_path(path,command_speed,cfg,.1)
        command_speed=min(.35,cap,command_speed+dt)
        steering=float(np.clip(want,steering-3*dt,steering+3*dt));peak=max(peak,abs(steering))
        # 1~5 s 사이 바퀴는 목표속도로 도나 지면 진행은1/5인 합성 외란.
        # 이후 이상 접지로 복귀. 실제 모터·마찰·제동·방지턱 모델이 아니다.
        ground_speed=command_speed*(.2 if slip and 1.<=t<5. else 1.)
        yaw_rate=ground_speed*math.tan(steering)/cfg.wheelbase_m
        new_pose=bicycle_step(*pose,ground_speed,steering,dt,cfg.wheelbase_m)
        yaw_step=yaw_rate*dt
        # 모터축 이동은 과대, gyro 회전은 이상적으로 맞는 관측 조합이다.
        moved=command_speed*dt
        if abs(yaw_rate)>1e-12:
            radius=command_speed/yaw_rate
            dx=radius*math.sin(yaw_step);dy=radius*(1-math.cos(yaw_step))
        else:dx,dy=moved,0.
        dx+=cfg.wheelbase_m/2*(math.cos(yaw_step)-1)
        dy+=cfg.wheelbase_m/2*math.sin(yaw_step)
        path=transform(path,dx,dy,yaw_step)
        wall=np.array([[fit[2],fit[0]*fit[2]+fit[1]],[fit[3],fit[0]*fit[3]+fit[1]]])
        wall=transform(wall,dx,dy,yaw_step)
        center_y=pose[1]+cfg.wheelbase_m/2*math.sin(pose[2])
        if fresh:
            centered=centered+1 if abs(center_y)<=.025 and abs(pose[2])<=.08 else 0
            if centered>=2 and first_center is None:first_center=t
        rows.append(dict(t=t,rear_pose=list(pose),center_y=center_y,steering=steering,
            wheel_speed=command_speed,ground_speed=ground_speed,refresh=applied))
        pose=new_pose
    return dict(passed=first_center is not None,centered_s=first_center,refreshes=refreshes,
        peak_steering_rad=peak,final_center_y_m=rows[-1]['center_y'],final_yaw_rad=pose[2],
        scope='ideal current wall and bicycle component only; no guard, collisions, rays, MCU or dynamics'),rows


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    files=[Path(__file__),source,*(ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')]
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    results={};trajectories={}
    for slip in (False,True):
        for enabled in (False,True):
            name=('slip' if slip else 'ideal')+('_refresh' if enabled else '_original')
            results[name],trajectories[name]=run_case(enabled,slip)
    unchanged=all(sha(ROOT/p)==v for p,v in hashes.items())
    summary=dict(results=results,source_unchanged=unchanged,source_sha256=hashes,
        ideal_commands_identical=trajectories['ideal_original']==trajectories['ideal_refresh'])
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    (output/'trajectories.json').write_text(json.dumps(trajectories,ensure_ascii=False))
    print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'}))
    raise SystemExit(0 if unchanged else 1)
