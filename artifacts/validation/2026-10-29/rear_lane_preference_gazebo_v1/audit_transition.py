"""실제 취득 광선과 평가 전용 자세로 후방 사각 전이를 대조한다.

차량 식별/제어 코드가 아니다. 상자 actor의 자세 사이 보간과 거리 일치에는 오차가 있으며,
반환 0을 사각 또는 자유 공간으로 단정하지 않는다. 마스크 안 기하와 반환 일치를 분리한다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import thaw


def poses_at(rows, times):
    rows = np.asarray(rows, dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 4 or len(rows) < 2 or not np.all(np.isfinite(rows)):
        raise ValueError('invalid pose rows')
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('unordered pose rows')
    index = np.searchsorted(rows[:, 0], times, side='right')-1
    index = np.clip(index, 0, len(rows)-2)
    lo, hi = rows[index], rows[index+1]
    valid = ((times >= lo[:, 0]-1e-9) & (times <= hi[:, 0]+1e-9)
             & (hi[:, 0]-lo[:, 0] <= .05+1e-9))
    ratio = (times-lo[:, 0])/(hi[:, 0]-lo[:, 0])
    yaw_delta = np.arctan2(np.sin(hi[:, 3]-lo[:, 3]), np.cos(hi[:, 3]-lo[:, 3]))
    return np.column_stack((lo[:, 1:3]+ratio[:, None]*(hi[:, 1:3]-lo[:, 1:3]),
                            lo[:, 3]+ratio*yaw_delta)), valid


def box_distances(origin, direction, actor, length=.2, width=.15):
    """각 광선의 시각에 존재하는 actor 로컬 AABB에 대한 slab 교차 거리."""
    delta = origin-actor[:, :2]
    c, s = np.cos(actor[:, 2]), np.sin(actor[:, 2])
    p = np.column_stack((c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1]))
    d = np.column_stack((c*direction[:, 0]+s*direction[:, 1],
                        -s*direction[:, 0]+c*direction[:, 1]))
    near, far = np.full(len(p), -np.inf), np.full(len(p), np.inf)
    valid = np.ones(len(p), dtype=bool)
    for axis, half in enumerate((length/2, width/2)):
        parallel = np.abs(d[:, axis]) < 1e-12
        valid &= ~(parallel & (np.abs(p[:, axis]) > half))
        denom = np.where(parallel, 1., d[:, axis])
        a, b = (-half-p[:, axis])/denom, (half-p[:, axis])/denom
        near = np.maximum(near, np.where(parallel, -np.inf, np.minimum(a, b)))
        far = np.minimum(far, np.where(parallel, np.inf, np.maximum(a, b)))
    valid &= far >= np.maximum(near, 0.)
    return np.where(valid, np.maximum(near, 0.), np.inf)


def scan_row(raw, motion, rear_half, ego_rows, actor_rows, reference):
    n = len(raw['ranges'])
    first = raw['sec']+raw['nanosec']*1e-9
    times = first+np.arange(n)*raw['time_increment']
    ego, ego_valid = poses_at(ego_rows, times)
    actor, actor_valid = poses_at(actor_rows, times)
    angle = raw['angle_min']+np.arange(n)*raw['angle_increment']
    wrapped = np.arctan2(np.sin(angle), np.cos(angle))
    mask = np.abs(wrapped) > math.pi-math.radians(rear_half)
    c, s = np.cos(ego[:, 2]), np.sin(ego[:, 2])
    ox, oy = motion['lidar_x_m'], motion['lidar_y_m']
    origin = ego[:, :2]+np.column_stack((c*ox-s*oy, s*ox+c*oy))
    bearing = ego[:, 2]+motion['lidar_yaw_rad']+angle
    direction = np.column_stack((np.cos(bearing), np.sin(bearing)))
    expected = box_distances(origin, direction, actor)
    potential = np.isfinite(expected) & (expected >= raw['range_min']) & (expected <= raw['range_max'])
    observed = np.asarray(raw['ranges'], dtype=float)
    # 진단용 반환 귀속 허용치. 제어/주행 판정에는 사용하지 않는다.
    with np.errstate(invalid='ignore'):
        matched = potential & np.isfinite(observed) & (np.abs(observed-expected) <= .025)
    ep, ev = poses_at(ego_rows, np.array([reference]))
    ap, av = poses_at(actor_rows, np.array([reference]))
    delta = ap[0, :2]-ep[0, :2]
    longitudinal = float(np.dot(delta, [math.cos(ep[0, 2]), math.sin(ep[0, 2])]))
    valid = bool(np.all(ego_valid & actor_valid) and ev[0] and av[0])
    visible_hits = int(np.sum(matched & ~mask))
    unmasked_opportunities = int(np.sum(potential & ~mask))
    masked_opportunities = int(np.sum(potential & mask))
    if not valid:
        state = 'invalid_pose_coverage'
    elif abs(longitudinal) <= .20:
        state = 'alongside'
    elif longitudinal > .20:
        state = 'ahead'
    elif visible_hits:
        state = 'behind_visible'
    elif masked_opportunities and not unmasked_opportunities and np.linalg.norm(delta) <= 1.5:
        state = 'blind'
    else:
        state = 'behind_unmatched'  # 가림/거리오차/누락은 마스크 사각으로 간주하지 않는다.
    return dict(first_s=first, end_s=float(times[-1]), reference_s=reference, state=state,
                longitudinal_m=longitudinal, distance_m=float(np.linalg.norm(delta)),
                visible_matched_returns=visible_hits, unmasked_geometric_rays=unmasked_opportunities,
                masked_geometric_rays=masked_opportunities, pose_coverage_valid=valid)


def ordered(sequence, required):
    index = 0
    for state in sequence:
        if index < len(required) and state == required[index]:
            index += 1
    return index == len(required)


def audit(output):
    report = json.loads((output/'report.json').read_text(encoding='utf-8'))
    actors = json.loads((output/'additional_actors.json').read_text(encoding='utf-8'))
    ego = [json.loads(x) for x in (output/'ego_poses.jsonl').read_text().splitlines()]
    actor = actors['poses']['eval_e3_side']
    first = report['case_window']['green_sim_s']
    last = first+report['case_window']['duration_s']
    rows, modes, holds, seen = [], Counter(), Counter(), set()
    with (output/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        header = json.loads(next(f))['data']
        for line in f:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            data = thaw(event['data']); inputs = data['input']
            if not first <= inputs['reference_s'] <= last:
                continue
            detail = data['output'][2]
            modes[str(detail.get('selector_mode'))] += 1
            keep = detail.get('rear_lane_keep') or {}
            holds[str(keep.get('reason'))] += 1
            raw = inputs['scan']; key = (raw['sec'], raw['nanosec'])
            if key in seen:
                continue
            seen.add(key)
            rows.append(scan_row(raw, header['motion_config'], inputs['rear_half_angle_deg'],
                                 ego, actor, inputs['reference_s']))
    transitions = []
    for row in rows:
        if not transitions or row['state'] != transitions[-1]['state']:
            transitions.append({**row, 'elapsed_s': row['reference_s']-first})
    sequence = [r['state'] for r in rows]
    realized = ordered(sequence, ['alongside', 'behind_visible', 'blind', 'behind_visible', 'ahead'])
    # 재접근이 한 스캔 만에 alongside로 넘어가는 경우도 사전 등록대로 포함한다.
    realized |= ordered(sequence, ['alongside', 'behind_visible', 'blind', 'alongside', 'ahead'])
    paths = [Path(__file__), output/'report.json', output/'ego_poses.jsonl',
             output/'additional_actors.json', output/'follow_decisions.jsonl']
    return dict(scope='evaluation-only sequential-ray attribution; 25mm match tolerance is diagnostic, not controller clearance',
                source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
                candidate=header['follow_config'].get('rear_lane_keep_trial', False),
                scenario_realized=bool(realized), scans=len(rows), state_counts=dict(Counter(sequence)),
                all_pose_coverage_valid=bool(rows) and all(r['pose_coverage_valid'] for r in rows),
                transitions=transitions, selector_modes=dict(modes), lane_keep_reasons=dict(holds),
                registered_order=['alongside','behind_visible','blind','behind_visible or alongside','ahead'],
                limitation='box geometry attribution and sampled pose interpolation; no general vehicle detection or hidden-car absence claim')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(args.output.resolve())
    target = args.output/'rear_transition_audit.json'
    with target.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
