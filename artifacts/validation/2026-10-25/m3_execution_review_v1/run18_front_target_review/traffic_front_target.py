"""복수 차량의 순간 전방 순서 보조 평가. 정답 자세는 평가 전용이며 원본 판정을 바꾸지 않는다.

현재 자차 방향의 직선 차체 폭과 겹치는, 차체 전체가 앞에 있는 actor를 고른다.
곡선 경로/차선의 장래 선행차 식별이 아니며 제어기·보호층으로 전달하지 않는다.
"""
from bisect import bisect_right
import argparse
import hashlib
import json
import math
from pathlib import Path

import g1_traffic_eval as ev


def front_target_at(ego, actors, time_s, *, max_pose_age_s=ev.POSE_MAX_GAP_S):
    if not math.isfinite(time_s) or not math.isfinite(max_pose_age_s) or max_pose_age_s <= 0:
        raise ValueError('invalid evaluation time/age')
    streams = {'ego': ego, **actors}
    if 'ego' in actors or not actors:
        raise ValueError('nonempty actor mapping without reserved ego key required')
    poses, ages = {}, {}
    for name, rows in streams.items():
        if not ev._pose_rows_valid(rows):
            return {'valid': False, 'reason': 'invalid_pose_stream', 'stream': name}
        i = bisect_right([r[0] for r in rows], time_s)-1
        if i < 0 or time_s-rows[i][0] > max_pose_age_s+1e-9:
            return {'valid': False, 'reason': 'missing_or_stale_pose', 'stream': name}
        poses[name], ages[name] = rows[i], time_s-rows[i][0]
    _, x, y, yaw = poses['ego']
    c, s = math.cos(yaw), math.sin(yaw)
    ego_poly = ev.rectangle(x, y, yaw, .20, .15)
    candidates, alongside, rows = [], [], {}
    for name in actors:
        _, ax, ay, heading = poses[name]
        poly = ev.rectangle(ax, ay, heading, .20, .15)
        local = [(c*(px-x)+s*(py-y), -s*(px-x)+c*(py-y)) for px, py in poly.exterior.coords[:-1]]
        lo, hi = min(p[0] for p in local), max(p[0] for p in local)
        left, right = min(p[1] for p in local), max(p[1] for p in local)
        overlap = left <= .075 and right >= -.075
        ahead = lo >= .10
        beside = not ahead and hi >= -.10
        row = {'longitudinal_interval_m': [lo, hi], 'lateral_interval_m': [left, right],
               'heading_corridor_overlap': overlap, 'fully_ahead': ahead,
               'body_gap_m': float(ego_poly.distance(poly)), 'longitudinal_gap_m': lo-.10,
               'pose_age_s': ages[name]}
        rows[name] = row
        if overlap and ahead:
            candidates.append((lo, name))
        if beside:
            alongside.append(name)
    chosen = min(candidates)[1] if candidates else None
    return {'valid': True, 'reference_s': time_s, 'ego_pose_age_s': ages['ego'],
            'scope': 'instantaneous straight heading corridor; not curved-path prediction or control input',
            'front_actor': chosen, 'front_body_gap_m': None if chosen is None else rows[chosen]['body_gap_m'],
            'alongside_actors': alongside, 'actors': rows}


def evaluate_run(run):
    names = ('report.json', 'ego_poses.jsonl', 'actor_poses.jsonl', 'additional_actors.json')
    before = {name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in names}
    report = json.loads((run/'report.json').read_text(encoding='utf8'))
    load = lambda name: [json.loads(line) for line in (run/name).read_text(encoding='utf8').splitlines()]
    actors = {'primary_lead': load('actor_poses.jsonl'),
              **json.loads((run/'additional_actors.json').read_text(encoding='utf8'))['poses']}
    end = report['case_window']['end_sim_s']
    result = front_target_at(load('ego_poses.jsonl'), actors, end)
    result.update(original_passed=report['passed'], original_final_gap_gate=report['evaluation']['gates'].get('final_gap'),
                  original_primary_gap_m=report['evaluation']['progress']['sampled_gap_at_window_end_m'],
                  registered_upper_gap_m=1.2, input_sha256=before,
                  interpretation='diagnostic comparison only; original run remains unchanged and failed gates remain')
    if result['valid'] and result['front_actor'] is not None:
        result['front_gap_within_registered_upper_bound'] = result['front_body_gap_m'] <= 1.2
    else:
        result['front_gap_within_registered_upper_bound'] = None
    result['inputs_unchanged'] = all(hashlib.sha256((run/name).read_bytes()).hexdigest() == h for name,h in before.items())
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(a.run.resolve()):
        raise ValueError('new output outside original run required')
    result = evaluate_run(a.run)
    a.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['valid'] and result['inputs_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
