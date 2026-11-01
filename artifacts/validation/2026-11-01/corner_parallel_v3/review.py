"""코너 병렬 장면의 실제 성립과 주행 성과를 분리하는 사후 평가."""
from hashlib import sha256
import importlib.util
import json
import math
from pathlib import Path
import sys
from zipfile import ZipFile
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];RUN=HERE/'run1'
sys.path.insert(0,str(ROOT/'scripts'))
from g1_traffic_eval import POSE_MAX_GAP_S


def longest(times,mask,step=.02):
    start=None;best=dict(duration_s=0.,start_s=None,end_s=None)
    for i,valid in enumerate([*mask,False]):
        if valid and start is None:start=i
        if not valid and start is not None:
            end=times[i-1]+step;duration=end-times[start]
            if duration>best['duration_s']:best=dict(duration_s=duration,start_s=float(times[start]),end_s=float(end))
            start=None
    return best


def array(rows,first,last):
    points=np.asarray(rows,dtype=float)
    if (points.ndim!=2 or points.shape[1]!=4 or not np.all(np.isfinite(points))
            or np.any(np.diff(points[:,0])<=0) or points[0,0]>first or points[-1,0]<last):
        raise ValueError('finite ordered covered pose stream required')
    low=max(0,np.searchsorted(points[:,0],first,side='right')-1)
    high=min(len(points),np.searchsorted(points[:,0],last,side='left')+1)
    gap=float(np.max(np.diff(points[low:high,0])))
    if gap>POSE_MAX_GAP_S+1e-9:raise ValueError('uncovered pose interval')
    points[:,3]=np.unwrap(points[:,3])
    return points,gap


def at(points,times):
    return np.column_stack([np.interp(times,points[:,0],points[:,k]) for k in (1,2,3)])


def review():
    report=json.loads((RUN/'report.json').read_bytes())
    green=report['green_observed_sim_s'];end=min(green+17.,report['case_window']['end_sim_s'])
    ego,ego_gap=array([json.loads(line) for line in (RUN/'ego_poses.jsonl').read_bytes().splitlines()],green,end)
    traffic=json.loads((RUN/'additional_actors.json').read_bytes())
    side,side_gap=array(traffic['poses']['eval_corner_side'],green,end)
    times=np.arange(green+.10,end-.019,.02)
    own=at(ego,times);other=at(side,times)
    own_speed=np.linalg.norm(own[:,:2]-at(ego,times-.1)[:,:2],axis=1)/.1
    side_speed=np.linalg.norm(other[:,:2]-at(side,times-.1)[:,:2],axis=1)/.1
    delta=other[:,:2]-own[:,:2];c=np.cos(own[:,2]);s=np.sin(own[:,2])
    longitudinal=c*delta[:,0]+s*delta[:,1];lateral=-s*delta[:,0]+c*delta[:,1]
    initial_yaw=at(ego,[green])[0,2]
    angle=lambda x:np.abs(np.arctan2(np.sin(x),np.cos(x)))*180/math.pi
    turning=angle(own[:,2]-initial_yaw)
    aligned=angle(other[:,2]-own[:,2])<=45.
    beside=(np.abs(longitudinal)<=.30)&(np.abs(lateral)>=.15)&(np.abs(lateral)<=.45)
    moving=(own_speed>=.10)&(side_speed>=.10)
    curved=(turning>=30.)&(turning<=150.)
    presence=longest(times-green,beside&aligned&moving)
    corner=longest(times-green,beside&aligned&moving&curved)
    detail=[dict(elapsed_s=float(t-green),longitudinal_m=float(x),lateral_m=float(y),
                 ego_speed_mps=float(v),side_speed_mps=float(w),turn_deg=float(a),
                 corner_parallel=bool(ok)) for t,x,y,v,w,a,ok in
            zip(times,longitudinal,lateral,own_speed,side_speed,turning,beside&aligned&moving&curved)]
    with (HERE/'scene_rows.jsonl').open('x',encoding='utf-8') as f:
        for row in detail:f.write(json.dumps(row)+'\n')
    source=HERE.parent/'mixed_course_v1/dense_progress.py'
    spec=importlib.util.spec_from_file_location('corner_dense_progress',source)
    dense=importlib.util.module_from_spec(spec);spec.loader.exec_module(dense)
    with ZipFile(RUN/'source_snapshot.zip') as archive:
        changed=[name for name,value in report['source_sha256'].items()
                 if name not in archive.namelist() or sha256(archive.read(name)).hexdigest()!=value]
    return dict(scope='평가 정답으로 장면 성립만 판별. 제어 입력/실물 증명이 아니며 원래 실패를 바꾸지 않음.',
        input_sha256={n:sha256((RUN/n).read_bytes()).hexdigest() for n in
                      ('report.json','ego_poses.jsonl','additional_actors.json')},
        script_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
        raw_run_valid=report['run_valid'],raw_passed=report['passed'],
        raw_failed_gates=[k for k,v in report['gates'].items() if not v],
        window=[green,end],max_pose_gap_s=dict(ego=ego_gap,side=side_gap),
        sample_interval_s=.02,speed_difference_window_s=.10,
        moving_beside=presence,corner_moving_beside=corner,
        corner_parallel_realized=corner['duration_s']>=.30,
        ego_ever_turned_30deg=bool(np.any(curved)),dense_progress=dense.audit(RUN),
        wall_collision_samples=report['wall_collision_samples'],
        additional_evaluation=report['additional_actors'],original_evaluation=report['evaluation'],
        source_unchanged=report['source_unchanged'],snapshot_mismatches=changed,
        stop_checks=report['stop_checks'],shutdown_clean=report['shutdown_clean'],
        remaining_test_pids=report['remaining_test_pids'])


if __name__=='__main__':
    result=review()
    with (HERE/'review.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:result[k] for k in ('raw_run_valid','raw_passed','moving_beside',
                      'corner_moving_beside','corner_parallel_realized','snapshot_mismatches')},ensure_ascii=False))
