"""R0+ 제한 선별. 정지 합성 광선 + 이미 끝난 G1 입력의 반사실 판단.

새 폐루프/가상 센서 주행이 아니며 실제 주행선·간격 개선을 주장하지 않는다.
"""
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from g0_geometry import TruthBox, Footprint, cast_rays
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_path import pursuit_command
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.r0_plus import R0Plus
from arena_autonomy.follow_trace import restore_call, thaw, replay_events
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.scene_motion import motion_scene


def static_screen():
    walls = [TruthBox(0., y, .4, Footprint(8., .05), label=label)
             for y, label in ((.45, 'left_wall'), (-.45, 'right_wall'))]
    walls.append(TruthBox(4., 0., .4, Footprint(.05, .9), label='end_wall'))
    cases = [('clear', []), ('front_stopped', [TruthBox(.45, 0., .12)]),
             ('front_far', [TruthBox(.95, 0., .12)]),
             ('side_parallel', [TruthBox(0., .25, .12)]),
             ('side_front', [TruthBox(.2, .25, .12)]),
             ('side_two_near', [TruthBox(0., .25, .12), TruthBox(.25, .25, .12)]),
             ('front_corner', [TruthBox(.35, .08, .12, yaw_rad=.6)]),
             ('side_chain', [TruthBox(x, .25, .12) for x in (-.1, .11, .32, .53)]),
             ('two_sides', [TruthBox(.2, y, .12) for y in (-.25, .25)]),
             ('side_near_wall', [TruthBox(.2, .37, .12)]),
             ('side_tilted', [TruthBox(.2, .25, .12, yaw_rad=-.6)])]
    result = []
    for name, actors in cases:
        raw = cast_rays(walls+actors)
        fixed = LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate',
            occupancy_timing_mode='per_ray_candidate', corridor_fallback_enabled=True))
        plus = R0Plus()
        # 自車·相手 모두 정지한 같은 장면. 서로 다른 시각의 반복 관측을 4회 제공.
        from arena_autonomy.lidar_motion import MotionHistory
        h = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
        for t in np.arange(0., 1.001, .01):
            h.add_mean_wheel(float(t), 0.)
            h.add_gyro(float(t), 0.)
        for now in (.4, .5, .6, .7):
            scene = scene_from_scan(raw['ranges'], angle_min=-math.pi, angle_increment=math.tau/500,
                range_min=.05, range_max=12., acquired_at_s=[now-.1]*500, t_ref_s=now)
            g1 = fixed.command(scene, h, 0., 'left')
            rp = plus.command(scene, h, 0., 'left')
        r0 = pursuit_command(scene.occupied_endpoints_xy_m, 'left', 0., wheelbase=.135, max_speed=1., scan_age=.1)
        def brief(value):
            speed, steering, detail = value
            return {'speed_mps': speed, 'steering_rad': steering, 'reason': detail['reason'],
                'behavior': detail.get('behavior'), 'path_wall': detail.get('path_wall'),
                'swept_clearance_m': detail.get('swept_clearance_m')}
        result.append({'case': name, 'r0': brief(r0), 'r0_plus': brief(rp), 'g1_fixed': brief(g1),
            'visible_actor_rays': sum(label == 'target' and not masked for label, masked in zip(raw['truth_labels'], raw['masked']))})
    return result


def recorded_screen(path):
    events = [json.loads(line) for line in path.read_text().splitlines()]
    baseline = replay_events(events)
    if not baseline['passed']:
        raise RuntimeError('saved G1 baseline no longer matches')
    header = events[0]['data']
    motion = MotionConfig(**header['motion_config'])
    cfg = replace(FollowConfig(**header['follow_config']), boundary_evidence_mode='connected',
                  corridor_fallback_enabled=False, recent_observation_enabled=False)
    plus = R0Plus(cfg)
    reasons, comparison, examples = Counter(), Counter(), []
    for event in events[1:]:
        if event['kind'] == 'reset':
            plus.reset()
        elif event['kind'] == 'call':
            data, scan, h = restore_call(event['data']['input'], motion)
            try:
                s = motion_scene(scan, h, data['reference_s'], timing_verified=data['timing_verified'],
                    epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                out = plus.command(s, h, data['speed_mps'], data['side'], data['previous_steering_rad'], data['dt_s'])
            except (ValueError, MotionUnavailable) as error:
                plus.reset()
                out = (0., 0., {'reason': 'input_stop', 'error': str(error)})
            old = thaw(event['data']['output'])
            reasons[out[2]['reason']] += 1
            key = f"g1_{'move' if old[0] > 0 else 'hold'}_plus_{'move' if out[0] > 0 else 'hold'}"
            comparison[key] += 1
            if old[0] > 0 and out[0] == 0 and len(examples) < 8:
                examples.append({'seq': event['seq'], 'reference_s': data['reference_s'], 'g1_speed': old[0],
                    'plus_reason': out[2]['reason'], 'opposite_reason': out[2].get('opposite_reason')})
    return {'input': path.relative_to(ROOT).as_posix(), 'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'g1_original_calls_matched': baseline['matched'], 'reasons': dict(reasons),
            'comparison': dict(comparison), 'examples': examples,
            'scope': 'G1 trajectory input only; R0+ would change motion and future scans, not a closed-loop result'}


if __name__ == '__main__':
    if (OUT/'report.json').exists():
        raise FileExistsError('preserve existing screen')
    files = [Path(__file__), ROOT/'src/arena_autonomy/arena_autonomy/r0_plus.py', ROOT/'tests/test_r0_plus.py']
    previous = json.loads((OUT.parent/'e3_harness_v1/source_manifest.json').read_text())
    files += [ROOT/p for p in previous]
    files = sorted(set(files))
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in files:
            archive.write(p, p.relative_to(ROOT))
    report = {'scope': 'candidate screen only, no ROS/Gazebo/production activation', 'source_sha256': hashes,
        'synthetic': static_screen(),
        'recorded': recorded_screen(OUT.parent/'e3_harness_v1/run/follow_decisions.jsonl')}
    (OUT/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'source_sha256'}, ensure_ascii=False, indent=2))
