"""결측-제동 교집합 후보의 저장 입력 비교. production 파일이나 ROS 출력은 변경하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
for directory in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/directory))
from arena_autonomy import partial_observation as policy_module
from arena_autonomy.follow_trace import replay_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.scene_motion import relative_poses
from probe_joint_missing_sweep import joint_missing_sweep

ORIGINAL_EVALUATE=policy_module.PartialObservationPolicy.evaluate
ORIGINAL_NEAR=policy_module.near_sweep
COUNTS=Counter()
DETAILS=[]
COST=[]


def evaluate_joint(self,scan,points,history,now,speed,steering=0.,previous_steering=0.,
                   wheelbase=.135,length=.20,width=.15):
    def near(points,speed,age,steering,previous_steering,wheelbase,length,width,margin):
        original=ORIGINAL_NEAR(points,speed,age,steering,previous_steering,wheelbase,length,width,margin)
        selected=np.flatnonzero(original)
        if not len(selected):return original
        source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        raw=np.asarray(scan.ranges,dtype=float)
        step=float(scan.angle_increment)
        angles=(scan.angle_min+np.arange(len(raw))*step+history.config.lidar_yaw_rad+math.pi)%math.tau-math.pi
        valid=np.isfinite(raw)&(raw>=scan.range_min)&(raw<=scan.range_max)
        unknown=np.flatnonzero(~valid&(np.abs(angles)<=math.radians(100)+1e-6))
        poses=relative_poses(history,source+unknown*scan.time_increment,now)
        directions=angles[unknown]+poses[:,2]
        c,s=np.cos(poses[:,2]),np.sin(poses[:,2]);sensor=history.config
        origins=poses[:,:2]+np.column_stack((c*sensor.lidar_x_m-s*sensor.lidar_y_m,
                                           s*sensor.lidar_x_m+c*sensor.lidar_y_m))
        required=max(.6,.16+speed*(now-source+.10)+speed*speed/4)
        # 기존 개략 검사에 남는 점만 계산한다. 범위 밖이면 기존 정지 쪽 판단을 유지한다.
        start=time.perf_counter()
        try:
            distance,_=joint_missing_sweep(np.asarray(points)[selected],origins,directions,step/2,
                required,speed,age,steering,previous_steering,wheelbase,length,width)
        except ValueError:
            COUNTS['outside_shadow_bounds']+=1
            return original
        COST.append((time.perf_counter()-start)*1000)
        rejected=selected[distance>margin+1e-10]
        output=original.copy();output[rejected]=False
        COUNTS['joint_calls']+=1
        COUNTS['broad_selected_points']+=len(selected)
        COUNTS['no_shared_location_points']+=len(rejected)
        if len(rejected) and 129.3<=now<=129.6:
            DETAILS.append(dict(t_s=now,margin_m=margin,source_first_s=source,
                previous_points_xy=np.asarray(points)[selected].tolist(),
                joint_distance_m=distance.tolist(),selected_original_indices=selected.tolist(),
                rejected_original_indices=rejected.tolist()))
        return output
    with patch.object(policy_module,'near_sweep',side_effect=near):
        return ORIGINAL_EVALUATE(self,scan,points,history,now,speed,steering,previous_steering,wheelbase,length,width)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--until',type=float,default=159.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    sources=[Path(__file__),ROOT/'scripts/probe_joint_missing_sweep.py',args.trace,
             ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py',
             ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py']
    before={str(p.resolve().relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    counts=Counter();reasons=Counter();changes=[];latched=[];call_ms=[]
    with args.trace.open() as stream:
        header=json.loads(next(stream))['data']
        follow=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
        with patch.object(policy_module.PartialObservationPolicy,'evaluate',evaluate_joint):
            for line in stream:
                row=json.loads(line)
                if row['kind']=='reset':follow.reset();counts['resets']+=1;continue
                if row['kind']!='call':continue
                data=row['data'];t=data['input']['reference_s']
                if t>args.until:break
                start=time.perf_counter();output=replay_call(follow,data['input'],motion)
                call_ms.append((time.perf_counter()-start)*1000);counts['calls']+=1
                old=data['output'];detail=output[2];partial=detail.get('partial_observation') or {}
                reasons[detail['reason']]+=1
                if partial.get('reason')=='known_obstacle_latched':
                    counts['latched_calls']+=1
                    if not latched or partial.get('latch_time_s')!=latched[-1]['latch_time_s']:
                        latched.append(dict(t_s=t,latch_time_s=partial.get('latch_time_s'),partial=partial))
                if not np.allclose(output[:2],old[:2],rtol=0,atol=1e-10):
                    counts['command_changed']+=1
                    changes.append(dict(t_s=t,baseline=old[:2],candidate=list(output[:2]),
                        baseline_reason=old[2]['reason'],reason=detail['reason'],
                        partial=partial,selector_reason=detail.get('selector_reason')))
    after={str(p.resolve().relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    cost=lambda a:dict(median=float(np.median(a)),p95=float(np.percentile(a,95)),maximum=float(max(a))) if a else None
    report=dict(scope='sensor-only open-loop geometry comparison; no ROS, no new driving or independent safety execution',
        unchanged_assumptions='3 cm + 2 m/s * previous-reference age; existing angle/speed/TTL/STOP gates',
        counts=dict(counts),joint_counts=dict(COUNTS),reasons=dict(reasons),
        first_change=changes[0] if changes else None,first_remaining_latch=latched[0] if latched else None,
        joint_wall_cost_ms=cost(COST),controller_wall_cost_ms=cost(call_ms),
        source_sha256=before,source_unchanged=before==after)
    for name,value in [('report',report),('changes',changes),('latches',latched),('first_contexts',DETAILS)]:
        (args.output/f'{name}.json').write_text(json.dumps(freeze(value),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(freeze({k:report[k] for k in ('counts','joint_counts','first_change','controller_wall_cost_ms','source_unchanged')})))
    if before!=after:raise RuntimeError('source changed during comparison')


if __name__=='__main__':main()
