#!/usr/bin/env python3
"""독립 ROSClock과 Python monotonic 시각을 기록. ROS 노드/토픽/시스템 시계 변경 없음."""
import argparse
import json
from pathlib import Path
import time
import uuid

from arena_vehicle_interface.clock_pair_audit import capture_pair, pair_dict
from audit_clock_pairs import SCHEMA


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=500)
    parser.add_argument('--period-ms',type=float,default=20.)
    args=parser.parse_args()
    if not 2<=args.samples<=10000 or not 1<=args.period_ms<=1000:
        parser.error('2..10000 samples and 1..1000 ms period required')
    if args.output.exists():parser.error('output must be new')
    # 의도적으로 독립 Clock만 읽는다. 이 결과는 다른 노드의 use_sim_time 검사나
    # 실제 센서 드라이버/Jetson의 시간 확인으로 확대하지 않는다.
    from rclpy.clock import ROSClock
    clock=ROSClock()
    capture_id=str(uuid.uuid4())
    host_epoch='process-'+capture_id
    ros_epoch='standalone-ros-clock-'+capture_id
    header=dict(kind='header',schema=SCHEMA,origin='pc_clock_probe',host_clock='time.monotonic_ns',
        host_epoch=host_epoch,ros_epoch=ros_epoch,
        source_context='standalone rclpy ROSClock without TimeSource; process epoch tokens, not OS boot identity; no sensor/node integration')
    completed=False
    count=0
    period_ns=round(args.period_ms*1_000_000)
    next_tick=time.monotonic_ns()
    with args.output.open('x',encoding='utf-8') as stream:
        stream.write(json.dumps(header,allow_nan=False)+'\n')
        try:
            for sequence in range(args.samples):
                remaining=next_tick-time.monotonic_ns()
                if remaining>0:time.sleep(remaining/1_000_000_000)
                pair=capture_pair(sequence,host_epoch,ros_epoch,monotonic_ns=time.monotonic_ns,
                    ros_now_ns=lambda:clock.now().nanoseconds,
                    read_mode=lambda:(clock.clock_type.name,clock.ros_time_is_active))
                stream.write(json.dumps(dict(kind='pair',**pair_dict(pair)),allow_nan=False)+'\n')
                count+=1
                # 지연 뒤 빠른 몰아쓰기 측정을 하지 않는다. 실제 간격은 기록에서 확인한다.
                next_tick=time.monotonic_ns()+period_ns
            completed=True
        finally:
            stream.write(json.dumps(dict(kind='footer',completed=completed,
                expected_count=args.samples,written_count=count),allow_nan=False)+'\n')
    print(json.dumps(dict(completed=completed,samples=count,output=str(args.output),
                          ros_graph_started=False,system_clock_changed=False)))


if __name__=='__main__':main()
