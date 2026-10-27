#!/usr/bin/env python3
"""저장 G1 센서 입력에서 새 보호 정책만 대조한다. 새 경로/주행 재생이 아니다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.partial_observation import PartialObservationPolicy


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    before=digest(args.capture)
    policy=PartialObservationPolicy()
    reasons=Counter();rows=[];costs=[];calls=0;unknown=0;allowed=0
    with args.capture.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
            elif event['kind']=='reset':policy.reset()
            elif event['kind']=='call':
                calls+=1
                data,scan,h=restore_call(event['data']['input'],motion)
                now=data['reference_s']
                audit=audit_front_observation(scan.ranges,scan.angle_min,scan.angle_increment,
                    scan.range_min,scan.range_max,motion.lidar_yaw_rad)
                start=time.perf_counter()
                try:
                    first=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
                    points,_,_=h.project(scan.ranges,scan.angle_min,scan.angle_increment,
                        scan.range_min,scan.range_max,first,scan.time_increment,now,'both',data['timing_verified'])
                    # 기존 1m/s 요청 상한을 사용한다. 저장된 STOP 명령을 새 목표로 쓰지 않는다.
                    result=policy.evaluate(scan,points,h,now,max(1.,data['speed_mps']),
                        data['previous_steering_rad'],data['previous_steering_rad'])
                except MotionUnavailable as error:
                    result={'allowed':False,'reason':str(error)}
                costs.append((time.perf_counter()-start)*1000)
                reasons[result['reason']]+=1
                if audit.reason!='ok':
                    unknown+=1;allowed+=bool(result['allowed'])
                    rows.append(dict(seq=event['seq'],time_s=now,strict_reason=audit.reason,**result))
    report=dict(scope='saved_sensor_policy_comparison_not_closed_loop',calls=calls,
        strict_missing_calls=unknown,partial_allowed_calls=allowed,reasons=dict(reasons),
        maximum_ms=max(costs),p95_ms=float(np.percentile(costs,95)),
        input_sha256=before,input_unchanged=before==digest(args.capture),rows=rows,
        source_sha256={p.name:digest(p) for p in (Path(__file__),
            Path(__file__).resolve().parents[1]/'src/arena_autonomy/arena_autonomy/partial_observation.py')})
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256')}))


if __name__=='__main__':main()
