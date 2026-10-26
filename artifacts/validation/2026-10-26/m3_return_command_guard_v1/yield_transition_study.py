"""최초 위협 스캔의 양보 전이 길이 분해. 기존 지지/조향 조건을 바꾸지 않는 개루프 비교."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/p) for p in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'tests')]
import arena_autonomy.m3_execution_trial as m3
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lateral_intent import offset_candidate, rectangle_corners, wheel_contact_corners


def study(name, deadline):
    path = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
    raw = path.read_bytes(); events = [json.loads(l) for l in raw.decode('utf-8').splitlines()]
    header = events[0]['data']
    ctl = m3.M3ExecutionFollow(FollowConfig(**header['follow_config']),
                             m3.ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    original = m3.plan_check; rows = []; current = {}; baseline = {}
    base_command = ctl.base.command

    def base(*args, **kw):
        out = base_command(*args, **kw); baseline['value'] = out; return out

    def plan(scene, frames, candidate, context, cfg, execution, speed):
        result = original(scene, frames, candidate, context, cfg, execution, speed)
        if candidate['name'] != 'yield' or rows:
            return result
        # 동일 최초 장면/목표/속도/이전각을 사용한다. 매 길이의 시험은 제어 상태를 바꾸지 않는다.
        state = m3.ego_state(context['frame'], cfg)
        for blend in (.20, .30, .40, .60):
            trial_cfg = replace(cfg, blend_min_m=blend)
            trial = m3.anchored(offset_candidate(context['frame'], state, candidate['d_target_m'], trial_cfg, 'yield'))
            check = original(scene, frames, trial, context, trial_cfg, execution, speed)
            previous_cfg = ctl.lateral_config
            try:
                ctl.lateral_config = trial_cfg
                command = ctl._evaluate_command('yield', trial['path'], scene, frames, current['history'], speed,
                    current['data']['previous_steering_rad'], current['data']['dt_s'],
                    context['follow_padding'], baseline['value']) if trial['valid'] else None
            finally:
                ctl.lateral_config = previous_cfg
            v = ctl.trial_speed_cap_mps
            progress = min(v*deadline, trial['progress'][-1])
            pose = np.array([np.interp(progress, trial['progress'], trial['poses'][:, j]) for j in range(3)])
            steer = np.interp(progress, trial['progress'], trial['steering_rad'])
            outline = np.vstack((rectangle_corners(pose[None, :], cfg.length_m, cfg.width_m).reshape(-1, 2),
                wheel_contact_corners(pose[None, :], np.array([steer]),
                    replace(cfg, contact_patch_length_m=2*cfg.wheel_radius_m)).reshape(-1, 2)))
            away_sign = np.sign(candidate['delta_m'])
            # 상대 물체의 종방향 겹침/이동은 제거한 횡단면 계산이다. 최소 실제 차간격 예측이 아니다.
            near_extent = np.max(-away_sign*outline[:, 1])
            rows.append({'blend_min_m': blend, 'actual_blend_m': trial['blend_m'], 'delta_m': trial['delta_m'],
                'valid': trial['valid'], 'problems': trial['problems'],
                'max_abs_steering_rad': trial['max_abs_steering_rad'], 'plan_result': check,
                'command': None if command is None else {k: command.get(k) for k in ('accepted', 'reason', 'target', 'steering')},
                'constant_cap_preview': {'speed_mps': v, 'elapsed_s': deadline, 'rear_progress_m': progress,
                    'body_lateral_away_m': float(away_sign*pose[1]),
                    'body_and_wheel_side_gain_m': float(cfg.width_m/2-near_extent),
                    'assumes_instant_acceleration_and_perfect_path_tracking': True}})
        return result

    ctl.base.command = base; m3.plan_check = plan
    try:
        for event in events[1:-1]:
            if event['kind'] == 'reset':
                ctl.reset(); continue
            data, scan, history = restore_call(event['data']['input'], motion)
            current.update(data=data, history=history)
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            except (ValueError, MotionUnavailable):
                ctl.reset(); continue
            out = ctl.command(scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'], data['dt_s'])
            if rows:
                break
    finally:
        m3.plan_check = original
    assert rows and abs(data['reference_s']-3.92) < 1e-6
    return {'first_threat_s': data['reference_s'], 'variants': freeze(rows),
            'input_sha256': hashlib.sha256(raw).hexdigest(), 'input_unchanged': raw == path.read_bytes()}


def main(out):
    if out.exists(): raise FileExistsError(out)
    sources = list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    results = {name: study(name, deadline) for name, deadline in
        [('run17_observed_release_closed_loop', .640), ('run21_right_encounter', .561)]}
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    out.mkdir(parents=True)
    report = {'scope': 'first-scan transition geometry only; no new drive, no baseline or protection changes',
              'source_sha256': hashes, 'source_unchanged': unchanged, 'results': results,
              'limits': ['Preview assumes instant acceleration and perfect tracking, not achievable trajectory proof.',
                        'Side extent ignores actor longitudinal overlap and motion; not actual separation prediction.',
                        'A shorter path remains unusable if observation or geometry rejects it.']}
    (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(results), flush=True)
    return 0 if unchanged and all(r['input_unchanged'] for r in results.values()) else 1


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    sys.exit(main(p.parse_args().output))
