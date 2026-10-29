"""설치한 G1/G2 ROS 노드의 명시 옵션·header·출력 경계를 확인한다.

센서 주행/물리/독립 보호의 동작 시험이 아니다. 격리된 ROS domain에서 센서 입력
없이 /drive 0 출력과 /drive/safe 직접 발행 부재, 설치 소스/설정 정합만 확인한다.
"""
from dataclasses import asdict
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import zipfile

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import arena_autonomy.bypass_handoff as impl
import arena_autonomy.g1_g2_node as node_impl
import arena_autonomy.bypass_entry_trial as entry_impl

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_case(out,candidate):
    trace=out/('candidate.jsonl' if candidate else 'default.jsonl')
    params=dict(use_sim_time=True,vision_execution='inline',g1_follow_enabled=True,
        g1_g2_simulation_trial=True,g1_trace_path=str(trace),lidar_compensation='both',
        motion_scan_timing_verified=True,motion_feedback_mode='drive_motor_shaft',motion_gear_ratio=15.,
        motion_wheel_radius_m=.0325,motion_rear_axle_x_m=-.0675,motion_lidar_x_m=.06,
        wheelbase_m=.135,lidar_x_m=.06,max_speed_mps=1.,control_rate_hz=50.,
        partial_observation_policy='bounded_local_v4',g1_boundary_evidence_mode='corner_distance_candidate',
        g1_occupancy_timing_mode='per_ray_candidate',g1_parallel_hold_trial=True,g1_pass_release_trial=True,
        g1_guard_curve_trial=True,g1_traffic_recovery_trial=True,g1_traffic_path_recovery_trial=True,
        g1_corridor_fallback_enabled=True)
    if candidate:
        params.update(g1_g2_preparation_policy='observed_entry_space_v1',
            g1_g2_persistence_window='trim_before_compare_v1',g1_g2_follow_entry_space=True,
            g1_g2_reacquire_policy='known_hold_v1')
    args=['--ros-args']
    for key,value in params.items():
        args += ['-p',key+':='+('true' if value is True else 'false' if value is False else str(value))]
    rclpy.init(args=args)
    node=observer=executor=None
    messages=[]
    try:
        node=node_impl.G1G2Node()
        observer=Node('handoff_option_audit',use_global_arguments=False)
        observer.create_subscription(AckermannDriveStamped,'/drive',lambda m:messages.append(m.drive.speed),10)
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(observer)
        # ROS 연결 대기 중 센서 입력은 없다. 주행 명령·센서 정답을 넣지 않는다.
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        node.control();node.stop()
        deadline=time.monotonic()+2.
        while not messages and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        cfg=asdict(node.follow.execution)
        safe_publishers=node.get_publishers_info_by_topic('/drive/safe')
        result=dict(candidate=candidate,parameters=params,execution_config=cfg,
            drive_messages=messages,drive_safe_publishers=len(safe_publishers),
            feedback_mode=node.motion.feedback_mode,feedback_topic=node.motion.feedback_topic,
            gear_ratio=node.motion.gear_ratio,wheel_radius_m=node.motion.history.config.wheel_radius_m)
        assert messages and all(v==0. for v in messages)
        assert not safe_publishers
        assert result['feedback_topic']=='/drive_motor/encoder' and result['gear_ratio']==15.
        assert cfg['follow_entry_space_enabled'] is candidate
        assert cfg['reacquire_policy']==('known_hold_v1' if candidate else 'positive_command_v0')
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if observer is not None:observer.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    events=[json.loads(line) for line in trace.read_text().splitlines()]
    assert events[0]['data']['execution_config']==cfg
    assert events[-1]['kind']=='footer' and events[-1]['data']['capture_errors']==0
    result.update(trace_sha256=digest(trace),footer=events[-1]['data'],passed=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='ros_options1')
    args=parser.parse_args()
    out=BASE/args.output;out.mkdir(exist_ok=False)
    sources={}
    for module in (impl,node_impl,entry_impl):
        actual=Path(module.__file__)
        assert '/install/' in actual.as_posix(),('source_tree_import_instead_of_installed',str(actual))
        expected=ROOT/'src/arena_autonomy/arena_autonomy'/actual.name
        assert digest(actual)==digest(expected),('stale_installed_source',str(actual))
        sources[str(expected.relative_to(ROOT))]=dict(sha256=digest(expected),installed=str(actual))
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for path in sources:z.write(ROOT/path,path)
        z.write(Path(__file__),Path(__file__).relative_to(ROOT))
    cases=[run_case(out,False),run_case(out,True)]
    report=dict(scope='installed ROS parameters/header/no-input zero output only; no driving or guard-chain validation',
        ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),sources=sources,cases=cases,
        source_unchanged=all(digest(ROOT/p)==d['sha256'] for p,d in sources.items()))
    assert report['source_unchanged']
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
