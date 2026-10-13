#!/usr/bin/env python3
"""G2 후방 가림 이후 이동 가능 영역의 평가 전용 하한 계산.

마지막 차량 외곽/자세는 정답을 사용한 낙관적 입력이다. 실제 인식/추적 아님.
상대차 중심은 지정 탐색속도 이하로 아무 방향 이동, yaw는 자유라고 가정한다.
외곽은 반대각선 원으로 감싸며 speed bound를 실물 제한으로 확정하지 않는다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

from shapely.geometry import Point, Polygon

from g0_geometry import Footprint
from g2_geometry_probe import ROOT, numeric
from g2_visibility_probe import pose_at, sequential_scene


def reach_gap(center_distance_m, *, body_radius_m, speed_bound_mps, elapsed_s,
              pose_error_m=.03, ego_motion_bound_m=0.):
    values = [numeric(v) for v in (center_distance_m, body_radius_m, speed_bound_mps,
                                   elapsed_s, pose_error_m, ego_motion_bound_m)]
    distance,radius,speed,elapsed,error,ego=values
    if any(v<0 for v in values) or radius==0:
        raise ValueError('nonnegative uncertainty and positive radius required')
    return max(0.,distance-radius-speed*elapsed-error-ego)


def last_visible_acquisition(scene, labels):
    # 평가용 identity만 사용하며 mask/unknown을 관측으로 인정하지 않는다.
    return max((ray.acquired_at_s for ray,label in zip(scene.rays,labels)
                if label=='target' and ray.status=='return'), default=None)


def evaluate_future(rows, start_time, last_observed, speed_bound):
    if last_observed is None:
        return dict(reason='target_never_observed', minimum_gap_lower_m=None,
                    conditional_envelope_clear=False, driving_authorized=False)
    if not 0 <= last_observed <= start_time:
        raise ValueError('last observation cannot be future')
    points=[]
    shape=Footprint()
    radius=math.hypot(.20,.15)/2
    rear_radius=math.hypot((.20+.135)/2,.15/2)
    for row in rows:
        if row['t_s'] < start_time-1e-10:
            continue
        pose=pose_at(rows,row['t_s'])
        footprint=Polygon(shape.polygon(*pose))
        distance=footprint.distance(Point(1.5,.24))
        ego_bound=abs(row['applied_speed_mps'])*.02*(1+rear_radius*abs(math.tan(row['applied_steering_rad'])/.135))
        # 다음 20 ms의 상대 운동도 포함한다. 취득 시각을 reference로 갱신하지 않는다.
        elapsed=row['t_s']+.02-last_observed
        gap=reach_gap(distance,body_radius_m=radius,speed_bound_mps=speed_bound,
                      elapsed_s=elapsed,ego_motion_bound_m=ego_bound)
        points.append(dict(t_s=row['t_s'],gap_lower_m=gap,last_observed_age_s=elapsed))
    minimum=min((p['gap_lower_m'] for p in points),default=None)
    first=next((p for p in points if p['gap_lower_m']<.15),None)
    return dict(reason='evaluated' if points else 'no_future_trajectory',minimum_gap_lower_m=minimum,
                conditional_envelope_clear=minimum is not None and minimum>=.15,
                last_observed_s=last_observed,start_age_s=start_time-last_observed,
                checked_intervals=len(points),first_below_existing_gap=first,driving_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    data=args.trace.read_bytes()
    rows=json.loads(data)
    output.mkdir(parents=True,exist_ok=False)
    names=['scripts/g2_rejoin_envelope.py','scripts/g2_visibility_probe.py','scripts/g2_tracking_probe.py',
           'scripts/g2_geometry_probe.py','scripts/g0_geometry.py','tests/test_g2_rejoin_envelope.py',
           'src/arena_autonomy/arena_autonomy/local_scene.py']
    raw={name:(ROOT/name).read_bytes() for name in names}
    hashes={name:hashlib.sha256(value).hexdigest() for name,value in raw.items()}
    with zipfile.ZipFile(output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for name,value in raw.items(): bundle.writestr(name,value)
    # 0.1초마다 취득을 재현한다. 마지막 유효 광선 시각은 보이지 않으면 갱신하지 않는다.
    observations=[]
    last=None
    for i in range(1,int(rows[-1]['t_s']/.1)+1):
        scene,labels,_=sequential_scene(rows,i*.1,.12)
        acquired=last_visible_acquisition(scene,labels)
        if acquired is not None: last=acquired
        observations.append(dict(reference_s=scene.t_ref_s,last_visible_acquisition_s=last,
                                 currently_visible=acquired is not None))
    cases=[]
    for stage in (1.5,1.85,2.15,2.6):
        current=min(rows,key=lambda row:abs(row['rear_x_m']-stage))
        prior=[o for o in observations if o['reference_s']<=current['t_s']]
        observed=prior[-1]['last_visible_acquisition_s'] if prior else None
        for speed in (0.,.5,1.):
            cases.append(dict(stage_rear_x_m=stage,speed_bound_mps=speed,
                              **evaluate_future(rows,current['t_s'],observed,speed)))
    unchanged=all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==value for name,value in hashes.items())
    report=dict(scope='truth_identity_last_observation_reachable_disk_evaluation_only',cases=cases,
                scan_observations=observations,source_sha256=hashes,sources_unchanged=unchanged,
                input_unchanged=args.trace.read_bytes()==data,input_sha256=hashlib.sha256(data).hexdigest(),
                driving_authorized=False, limits=[
                    'True target center and identity are assumed known: no estimator/tracker implemented.',
                    'A circumradius bounds arbitrary target yaw; position error .03 m is provisional.',
                    '0/.5/1 m/s are exploratory bounds, not verified competitor limits.',
                    'Only the remainder of the stored trajectory is checked, not braking/abort or indefinite safety.',
                    'Future observations are not credited; an overlap is inability to certify, not a predicted collision.'])
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(cases=len(cases),conditional_clear=sum(c['conditional_envelope_clear'] for c in cases),
                         scans=len(observations),sources_unchanged=unchanged,driving_authorized=False)))
    return 0 if unchanged and report['input_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
