"""첫 랩 고속 표본에서 명령 부족과 모터·접지 부족을 구분한다(평가 전용)."""
import argparse
import json
import math
from pathlib import Path
import numpy as np

BASE=Path(__file__).resolve().parent


def audit(folder,start,end):
    with (folder/'trajectory.jsonl').open(encoding='utf-8') as f:
        rows=[r['dynamics'] for r in map(json.loads,f) if r.get('dynamics')
              and start<=r['dynamics']['sim_time_s']<=end and r['light']=='green']
    if not rows or any(b['sim_time_s']<=a['sim_time_s'] for a,b in zip(rows,rows[1:])):
        raise ValueError('차체·축 속도를 함께 비교할 고유한 시간 표본이 없다')
    fast=[r for r in rows if r['truth_longitudinal_mps']>=2.]
    def stats(group):
        if not group:return dict(samples=0)
        err=np.asarray([r['wheel_surface_speed_mps']-r['truth_longitudinal_mps'] for r in group])
        return dict(samples=len(group),wheel_minus_body_mps_median=float(np.median(err)),
            wheel_minus_body_abs_p95_mps=float(np.percentile(np.abs(err),95)),
            maximum_wheel_minus_body_abs_mps=float(max(np.abs(err))),
            motor_rpm_min=min(r['motor_speed_rad_s'] for r in group)*60/math.tau,
            motor_rpm_max=max(r['motor_speed_rad_s'] for r in group)*60/math.tau,
            positive_motor_demand_clipped_samples=sum(r['raw_demand_nm']>0 and
                r['limited_demand_nm']<r['raw_demand_nm']-1e-8 for r in group),
            positive_target_negative_motor_samples=sum(r['applied_target_mps']>0 and r['motor_speed_rad_s']<0 for r in group))
    return dict(run=folder.name,lap_window_s=[start,end],all=stats(rows),body_above_2_mps=stats(fast),
        scope='같은 Gazebo 동역학 메시지 내 모터축 환산 평균 바퀴 속도와 차체 종방향 실제 속도. 약 10 Hz 평가 표본이며 한 바퀴씩의 접지나 실물 전류를 증명하지 않는다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('metrics',type=Path)
    p.add_argument('out',type=Path);a=p.parse_args()
    metrics=json.loads(a.metrics.read_text(encoding='utf-8'))
    start=metrics['first_green_sample_s'];end=start+metrics['lap_s_from_first_green_sample']
    result=audit(a.folder,start,end)
    with a.out.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
