#!/usr/bin/env python3
"""독립 rclpy ROSClock의 합성 모드/시간 변화 검사. ROS 그래프·시스템 시계 미변경."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import zipfile

from arena_vehicle_interface.clock_pair_audit import capture_pair, pair_dict
from audit_clock_pairs import SCHEMA, audit_bytes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if args.output_dir.exists():parser.error('new output directory required')
    args.output_dir.mkdir(parents=True)
    import rclpy.clock
    import rclpy.time_source
    from rclpy.clock import ROSClock
    from rclpy.time_source import TimeSource
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    root=Path(__file__).resolve().parents[1]
    paths=['scripts/probe_clock_pair_ros.py','scripts/audit_clock_pairs.py',
           'scripts/audit_clock_exchanges.py','src/arena_vehicle_interface/arena_vehicle_interface/clock_pair_audit.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/clock_bounds.py','tests/test_clock_pair_audit.py']
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    hashes={p:digest(root/p) for p in paths}
    with zipfile.ZipFile(args.output_dir/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(root/p,p)
    cases={
        'inactive_system':[(False,None)]*3,
        'active_uninitialized':[(True,None)]*3,
        'active_advancing':[(True,1_000_000_000),(True,2_000_000_000),(True,3_000_000_000)],
        'active_paused':[(True,1_000_000_000)]*3,
        'active_rewind':[(True,2_000_000_000),(True,3_000_000_000),(True,1_000_000_000)],
        'activate_then_deactivate':[(False,None),(True,1_000_000_000),(False,None)],
        'activate_inside_read':[(False,None),(False,None),(True,1_000_000_000)],
    }
    results=[]
    for name,steps in cases.items():
        clock=ROSClock()
        source=TimeSource()  # node=None: /clock 구독·ROS 그래프를 만들지 않는다.
        source.attach_clock(clock)
        host_epoch='probe-host-'+name
        ros_epoch='probe-clock-'+name
        header=dict(kind='header',schema=SCHEMA,origin='synthetic',host_clock='time.monotonic_ns',
            host_epoch=host_epoch,ros_epoch=ros_epoch,
            source_context='isolated installed rclpy clock and TimeSource fixture; no node, DDS, sensor or system-clock modification')
        rows=[header]
        for i,(active,stamp) in enumerate(steps):
            source.ros_time_is_active=active
            if stamp is not None:source.clock_callback(Clock(clock=Time(nanoseconds=stamp).to_msg()))
            def ros_now():
                if name=='activate_inside_read' and i==1:
                    source.ros_time_is_active=True
                    source.clock_callback(Clock(clock=Time(nanoseconds=1_000_000_000).to_msg()))
                return clock.now().nanoseconds
            pair=capture_pair(i,host_epoch,ros_epoch,monotonic_ns=time.monotonic_ns,
                ros_now_ns=ros_now,read_mode=lambda:(clock.clock_type.name,clock.ros_time_is_active))
            rows.append(dict(kind='pair',**pair_dict(pair)))
            time.sleep(.001)
        rows.append(dict(kind='footer',completed=True,expected_count=3,written_count=3))
        data=('\n'.join(json.dumps(r,allow_nan=False) for r in rows)+'\n').encode()
        with (args.output_dir/(name+'.jsonl')).open('xb') as stream:stream.write(data)
        result=audit_bytes(data)
        checks={
            'record_integrity':result['record_integrity_valid'],
            'no_authority':all(result[key] is False for key in ('between_samples_continuity_verified',
                            'ros_stamp_verified','clock_mapping_applied','ready_to_drive')),
            'physical_mode':result['physical_time_source_observed']==(name=='inactive_system'),
        }
        if name!='inactive_system':checks['no_offset_candidate']=not result['constant_offset_compatible']
        if name in ('active_uninitialized','active_paused'):
            checks['pause_events']=result['sampled_ros_pause_sequences']==[1,2]
        if name=='active_uninitialized':checks['zero_events']=result['sampled_zero_ros_sequences']==[0,1,2]
        if name=='active_advancing':
            checks['no_pause_or_regression']=not(result['sampled_ros_pause_sequences'] or result['sampled_ros_regression_sequences'])
        if name=='active_rewind':checks['backward_event']=result['sampled_ros_regression_sequences']==[2]
        if name=='activate_then_deactivate':checks['mode_events']=result['sampled_mode_change_sequences']==[1,2]
        if name=='activate_inside_read':checks['inside_read_event']=result['sampled_mode_change_sequences']==[1]
        results.append(dict(name=name,checks=checks,passed=all(checks.values()),
                            input_sha256=hashlib.sha256(data).hexdigest(),audit=result))
    report=dict(cases=results,all_expectations_passed=all(r['passed'] for r in results),
        source_sha256=hashes,source_unchanged=all(digest(root/p)==v for p,v in hashes.items()),
        installed_sources=[dict(path=m.__file__,sha256=digest(m.__file__)) for m in (rclpy.clock,rclpy.time_source)],
        ros_graph_started=False,system_clock_changed=False,ready_to_drive=False)
    with (args.output_dir/'report.json').open('x',encoding='utf-8') as stream:
        json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k not in ('cases','source_sha256','installed_sources')}))
    return 0 if report['all_expectations_passed'] and report['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
