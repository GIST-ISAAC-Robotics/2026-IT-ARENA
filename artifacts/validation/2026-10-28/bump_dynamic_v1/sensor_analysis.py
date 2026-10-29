"""순차 광선의 취득 시각과 차체 자세를 대조하는 사후 분석."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args();out=args.output
    if (out/'sensor_analysis.json').exists():raise FileExistsError('preserve analysis')
    raw=[out/name for name in ('bump_pose_trace.json','bump_imu_trace.json','bump_scan_trace.json','trace.json')]
    poses,imu,scans,trace=[json.loads(p.read_text(encoding='utf-8')) for p in raw]
    attitude=json.loads((out/'bump_analysis.json').read_text())['near_attitude']
    t=np.array([r['time'] for r in attitude]);pitch=np.array([r['pitch_deg'] for r in attitude])
    scan_rows=[]
    for scan in scans:
        if not t[0]<=scan['time']<=t[-1]:continue
        angles=scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']
        r=np.array([float(v) for v in scan['ranges']]);keep=(np.abs(angles)<=math.radians(10))&np.isfinite(r)
        ids=np.flatnonzero(keep)
        if not len(ids):continue
        idx=ids[np.argmin(r[ids])];stamp=scan['time']+idx*scan['time_increment']
        scan_rows.append(dict(first_s=scan['time'],minimum_ray_index=int(idx),minimum_range_m=float(r[idx]),
            ray_acquired_s=stamp,ray_angle_deg=float(math.degrees(angles[idx])),
            pitch_at_ray_deg=float(np.interp(stamp,t,pitch)) if t[0]<=stamp<=t[-1] else None))
    im=[r for r in imu if t[0]<=r['time']<=t[-1]]
    tr=[r for r in trace if t[0]<=r['time']<=t[-1]]
    result=dict(scope='recorded simulated IMU/scan and evaluation attitude; not measured impact or detected ground labels',
        near_range=[float(t[0]),float(t[-1])],front_scan_minima=scan_rows,
        max_abs_gyro_xyz_rad_s=np.max(np.abs([r['angular_velocity'] for r in im]),axis=0).tolist(),
        acceleration_xyz_min_mps2=np.min([r['linear_acceleration'] for r in im],axis=0).tolist(),
        acceleration_xyz_max_mps2=np.max([r['linear_acceleration'] for r in im],axis=0).tolist(),
        actual_speed_min_mps=min(r['dynamics']['truth_longitudinal_mps'] for r in tr),
        actual_speed_max_mps=max(r['dynamics']['truth_longitudinal_mps'] for r in tr),
        safety_reasons=sorted(set(r['safety']['reason'] for r in tr)),
        source_sha256={str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in raw})
    (out/'sensor_analysis.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    fig,ax=plt.subplots(3,1,figsize=(9,8),sharex=True,layout='constrained')
    ax[0].plot(t,pitch,color='#0072b2');ax[0].set_ylabel('Pitch (deg)\npositive = nose down')
    ax[1].plot([r['time'] for r in tr],[r['dynamics']['truth_longitudinal_mps'] for r in tr],label='Body speed')
    ax[1].plot([r['time'] for r in tr],[r['safe_drive_speed_mps'] for r in tr],label='Protected command')
    ax[1].set_ylabel('Speed (m/s)');ax[1].legend()
    ax[2].plot([r['ray_acquired_s'] for r in scan_rows],[r['minimum_range_m'] for r in scan_rows],'o-',color='#d55e00')
    ax[2].set_ylabel('Front +/-10 deg\nminimum range (m)');ax[2].set_xlabel('Simulation time (s)')
    for axis in ax:axis.grid(alpha=.25)
    fig.suptitle('Published 50 mm bump, cautious trial | sensor/evaluation evidence')
    fig.savefig(out/'bump_sensors.png',dpi=150)
    print(json.dumps({k:v for k,v in result.items() if k not in ('front_scan_minima','source_sha256')}))

if __name__=='__main__':main()
