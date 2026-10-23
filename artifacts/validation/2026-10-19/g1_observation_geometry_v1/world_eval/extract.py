"""탐색용: 고정 반례 장면 추출(pickle). 운영/정답 입력 없음."""
import json, pickle, sys
from pathlib import Path
from arena_autonomy.follow_trace import restore_call, equal_output, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.observation_support import transport_scene
from arena_autonomy.lidar_motion import MotionUnavailable
ROOT = Path.cwd()
CAPTURE = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'
targets = set(int(x) for x in sys.argv[1].split(','))
out = {}
mism = 0; n = 0
for line in CAPTURE.open():
    event = json.loads(line); kind = event['kind']
    if kind == 'header':
        cfg = FollowConfig(**event['data']['follow_config']); motion = MotionConfig(**event['data']['motion_config'])
        controller = LocalFollow(cfg)
    elif kind == 'reset':
        controller.reset()
    elif kind == 'call':
        d, scan, h = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, h, d['reference_s'], timing_verified=d['timing_verified'],
                             epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
        output = controller.command(scene, h, d['speed_mps'], d['side'], d['previous_steering_rad'], d['dt_s'])
        n += 1
        if not equal_output(freeze(output), event['data']['output']): mism += 1
        if event['seq'] in targets:
            sources = [scene]
            for old in reversed(controller.observation_cache.scenes):
                if old.source_first_s == scene.source_first_s: continue
                try: sources.append(transport_scene(old, h, scene.t_ref_s))
                except MotionUnavailable: pass
            out[event['seq']] = dict(d={k: v for k, v in d.items()}, scene=scene, sources=sources, history=h,
                                     output=freeze(output), cfg=cfg, motion=motion)
            if len(out) == len(targets): break
print('replayed', n, 'mismatch', mism, 'found', sorted(out))
pickle.dump(out, open(sys.argv[2], 'wb'))
