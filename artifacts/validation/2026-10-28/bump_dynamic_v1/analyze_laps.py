"""여러 랩의 방지턱 방문을 2차원 위치와 시각으로 분리하는 사후 평가."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze import pose_metrics


def visits(poses, bump):
    """같은 종방향 좌표의 먼 코스를 통과 표본으로 섞지 않는다."""
    result=[]; current=[]; previous=None
    c,s=math.cos(bump['yaw_rad']),math.sin(bump['yaw_rad'])
    for pose in poses:
        stamp=pose['time'];p=pose['position']
        if not all(math.isfinite(v) for v in (stamp,p['x'],p['y'])):
            raise ValueError('invalid pose/time')
        if previous is not None and stamp<previous:
            raise ValueError('time reversal')
        previous=stamp
        dx,dy=p['x']-bump['x'],p['y']-bump['y']
        inside=abs(c*dx+s*dy)<=.45 and abs(-s*dx+c*dy)<=.45
        if not inside or (current and stamp-current[-1]['time']>.25):
            if current:result.append(current);current=[]
        if inside:current.append(pose)
    if current:result.append(current)
    return result


def sensor_window(group, bump, imu, scans, trace):
    crossing=pose_metrics(group,bump)
    attitude=crossing.pop('near_attitude')
    if not attitude:return dict(crossing=crossing,reason='no_near_body_samples')
    t=np.array([p['time'] for p in attitude]);pitch=np.array([p['pitch_deg'] for p in attitude])
    minima=[]
    for scan in scans:
        end=scan['time']+(len(scan['ranges'])-1)*scan['time_increment']
        if end<t[0] or scan['time']>t[-1]:continue
        r=np.asarray([float(v) for v in scan['ranges']])
        angle=scan['angle_min']+np.arange(len(r))*scan['angle_increment']
        stamps=scan['time']+np.arange(len(r))*scan['time_increment']
        valid=(abs(angle)<=math.radians(10))&np.isfinite(r)&(r>=scan['range_min'])&(r<=scan['range_max'])
        valid&=(stamps>=t[0])&(stamps<=t[-1])
        ids=np.flatnonzero(valid)
        if not len(ids):continue
        idx=ids[np.argmin(r[ids])]
        minima.append(dict(first_s=scan['time'],ray_acquired_s=float(stamps[idx]),
            minimum_ray_index=int(idx),minimum_range_m=float(r[idx]),
            pitch_at_ray_deg=float(np.interp(stamps[idx],t,pitch))))
    ir=[r for r in imu if t[0]<=r['time']<=t[-1]]
    tr=[r for r in trace if t[0]<=r['time']<=t[-1]]
    result=dict(crossing=crossing,near_range_s=[float(t[0]),float(t[-1])],attitude=attitude,
        minima=minima,trace=tr,imu_count=len(ir),trace_count=len(tr))
    if ir:
        result['max_abs_gyro_xyz_rad_s']=np.max(np.abs([r['angular_velocity'] for r in ir]),axis=0).tolist()
        result['acceleration_xyz_min_mps2']=np.min([r['linear_acceleration'] for r in ir],axis=0).tolist()
        result['acceleration_xyz_max_mps2']=np.max([r['linear_acceleration'] for r in ir],axis=0).tolist()
    if tr:
        speeds=[r['dynamics']['truth_longitudinal_mps'] for r in tr]
        result.update(actual_speed_min_mps=min(speeds),actual_speed_max_mps=max(speeds),
            safety_reasons=sorted({r['safety']['reason'] for r in tr}),
            progress_range_m=[tr[0]['progress_m'],tr[-1]['progress_m']])
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    out=args.output;target=out/'bump_visits.json'
    if target.exists():raise FileExistsError('preserve prior visit analysis')
    base=Path(__file__).resolve().parent
    scene_path=base.parents[1]/'2026-10-27/official_departure_v2/fixture/scene.json'
    raw=[out/n for n in ('bump_pose_trace.json','bump_imu_trace.json','bump_scan_trace.json','trace.json')]
    before={str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in raw}
    poses,imu,scans,trace=[json.loads(p.read_text(encoding='utf-8')) for p in raw]
    bump=json.loads(scene_path.read_text(encoding='utf-8'))['speed_bumps']['bumps'][0]
    groups=visits(poses,bump)
    results=[sensor_window(g,bump,imu,scans,trace) for g in groups]
    report=json.loads((out/'report.json').read_text(encoding='utf-8'))
    overlay=json.loads((out/'trial_overlay.json').read_text(encoding='utf-8'))
    brief=[]
    for index,result in enumerate(results,1):
        row={k:v for k,v in result.items() if k not in ('attitude','trace')};row['visit_index']=index
        brief.append(row)
        if not result.get('attitude'):continue
        att=result['attitude'];tr=result['trace'];minima=result['minima']
        fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True,layout='constrained')
        axes[0].plot([r['time'] for r in att],[r['pitch_deg'] for r in att]);axes[0].set_ylabel('Pitch (deg)\nnose down +')
        if tr:
            axes[1].plot([r['time'] for r in tr],[r['dynamics']['truth_longitudinal_mps'] for r in tr],label='Body speed')
            axes[1].plot([r['time'] for r in tr],[r['safe_drive_speed_mps'] for r in tr],label='Protected command')
            axes[1].legend()
        axes[1].set_ylabel('Speed (m/s)')
        axes[2].plot([r['ray_acquired_s'] for r in minima],[r['minimum_range_m'] for r in minima],'o-')
        axes[2].set_ylabel('Front +/-10 deg\nminimum range (m)');axes[2].set_xlabel('Simulation time (s)')
        for axis in axes:axis.grid(alpha=.25)
        fig.suptitle(f"Published 50 mm bump | visit {index} | {overlay['launch_arguments']['speed_profile']}")
        fig.savefig(out/f'bump_visit_{index}.png',dpi=150);plt.close(fig)
    unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest()==before[p.name] for p in raw)
    result=dict(scope='separate simulated 2D visits, geometric wheel crossing; not force/hardware proof',
        requested_laps=report.get('requested_laps'),base_passed=report.get('passed'),visits=brief,
        raw_sha256=before,raw_unchanged=unchanged,
        analysis_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),base/'analyze.py',scene_path)})
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(visits=len(brief),crossings=sum(r['crossing']['all_wheels_passed'] for r in brief),raw_unchanged=unchanged)))
    return 0 if unchanged else 1


if __name__=='__main__':raise SystemExit(main())
