"""같은 저장 명령/센서로 실제 독립 보호 control 메서드를 대조한다.

ROS 노드 수송/MCU/주행은 아니다. project_scan과 정책을 실제 사용한다.
"""
import importlib.util
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from builtin_interfaces.msg import Time
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.known_structure import structure_policy
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.scan_input import project_scan
from test_lidar_safety import node_double

A = Path(__file__).resolve().parent
P = A.parent/'path_side_gap_r164_v1'
spec = importlib.util.spec_from_file_location('runtime_safety_r165', A/'safety.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
trace = A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'
events = [json.loads(s) for s in trace.open()]
outputs = {r['seq']:r['output'] for r in map(json.loads, (P/'expected.jsonl').open())}
header = events[0]['data']; cfg = FollowConfig(**header['follow_config'])
motion = MotionConfig(**header['motion_config'])
rows = {}
for label in ('baseline', 'candidate'):
    if label == 'candidate':
        candidate.install()
    node, commands, statuses = node_double()
    node.wheelbase, node.length, node.width = cfg.wheelbase_m, cfg.length_m, cfg.width_m
    node.partial_policy = structure_policy(RecoveryPolicy, cfg.known_structure_sectors_json)(
        allow_narrow_forward=True, surface_translation_enabled=True,
        lateral_metric_enabled=True, oblique_metric_enabled=cfg.oblique_gap_trial)
    result = []
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            node.partial_policy.reset(); node.last_clock = None
        if event['kind'] != 'call':
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        now = data['reference_s']; ns = round(now*1e9)
        stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
        node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ns,to_msg=lambda:stamp))
        node.scan = scan
        node.command.header.stamp = stamp
        node.command.drive.speed, node.command.drive.steering_angle = outputs[event['seq']][:2]
        node.previous_safe_steering = data['previous_steering_rad']
        node.scan_wall = node.command_wall = time.monotonic()
        node.rear_blind_half_angle = data['rear_half_angle_deg']
        def project(raw):
            points, indices, _ = project_scan(raw, history, now, 'both', data['timing_verified'])
            return points, len(indices)
        node.motion = SimpleNamespace(history=history,project=project,
            verified=data['timing_verified'], counts={'resets': data['epoch']})
        LidarSafety.control(node)
        status = json.loads(statuses[-1].data)
        result.append(dict(seq=event['seq'], requested_mps=node.command.drive.speed,
            safe_mps=commands[-1].drive.speed, reason=status['reason'],
            partial=status.get('partial_observation')))
        assert 0 <= commands[-1].drive.speed <= node.command.drive.speed
    rows[label] = result
candidate.uninstall()
changes = [dict(seq=b['seq'],before=a['safe_mps'],after=b['safe_mps'],
                old_reason=a['reason'],reason=b['reason'],partial=b['partial'])
           for a,b in zip(rows['baseline'],rows['candidate']) if a['safe_mps'] != b['safe_mps']]
summary = dict(calls=len(rows['candidate']), stop_to_go=sum(r['before']<=0<r['after'] for r in changes),
    go_to_stop=sum(r['after']<=0<r['before'] for r in changes), changes=len(changes),
    scope='recorded acquisition snapshots and same command; actual independent control method; no ROS transport or live drive')
(A/'safety_comparison.json').write_text(json.dumps(dict(summary=summary,changes=changes),indent=2))
(A/'safety_rows.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(summary))
