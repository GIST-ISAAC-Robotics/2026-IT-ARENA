"""동일 2.0 실패 입력에서 열린 직선·코너 진입의 제어 목표를 비교한다. 자세는 평가에만 쓴다."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig

here = Path(__file__).resolve().parent
run = here.parent/'race_continuity_v1'/'solo_2p0_corrected'
trace = run/'follow_decisions.jsonl'
# 평가 시간창은 과거 주행에서 선택. 제어기에 거리·정답 자세·현재 지도 좌표를 넘기지 않는다.
regions = {'green': (10.7, 37.2), 'first_straight': (12.8, 14.8), 'approach': (35.4, 36.8)}
def fresh():
    return {'n': 0, 'lower': 0, 'higher': 0, 'old_positive_to_zero': 0, 'old_zero_to_positive': 0,
            'old_peak': 0., 'new_peak': 0., 'old_sum': 0., 'new_sum': 0., 'angle_change': 0}
by = {name: fresh() for name in regions}
off_mismatches = 0; matched = 0
with trace.open('rb') as stream:
    for line in stream:
        row = json.loads(line); val = row['data']
        if row['kind'] == 'header':
            cfg = FollowConfig(**val['follow_config']); motion = MotionConfig(**val['motion_config'])
            off, on = LocalFollow(cfg), LocalFollow(replace(cfg, blend_open_speed_by_length=True))
        elif row['kind'] == 'reset':
            off.reset(); on.reset()
        elif row['kind'] == 'call':
            t = val['input']['reference_s']
            if t > 37.2: break
            recorded = val['output']; prior = replay_call(off, val['input'], motion)
            new = replay_call(on, val['input'], motion)
            matched += 1
            off_mismatches += int(abs(recorded[0]-prior[0])>1e-8 or abs(recorded[1]-prior[1])>1e-8)
            for name, (start, end) in regions.items():
                if not start <= t <= end: continue
                b = by[name]; old = prior[0]; v = new[0]
                b['n'] += 1; b['lower'] += int(v < old-1e-8); b['higher'] += int(v > old+1e-8)
                b['old_positive_to_zero'] += int(v == 0. and old > 0.)
                b['old_zero_to_positive'] += int(v > 0. and old == 0.)
                b['old_peak'] = max(b['old_peak'], old); b['new_peak'] = max(b['new_peak'], v)
                b['old_sum'] += old; b['new_sum'] += v
                b['angle_change'] += int(abs(prior[1]-new[1]) > 1e-8)
for b in by.values():
    for prefix in ['old','new']:
        b[prefix+'_mean'] = b.pop(prefix+'_sum') / b['n']
with trace.open('rb') as stream: digest = hashlib.file_digest(stream,'sha256').hexdigest()
result = {'input_sha256':digest, 'matched_rows':matched, 'off_mismatches':off_mismatches,
          'regions_sim_s':regions, 'rows': by, 'scope':'fixed old sensor trajectory, not new vehicle behavior'}
print(json.dumps(result, indent=2), flush=True)
with (here/'replay.json').open('x') as f: json.dump(result, f, indent=2)
assert off_mismatches == 0 and by['first_straight']['new_peak'] >= 1.99
assert by['approach']['lower'] > 0 and by['green']['higher'] == 0
assert not by['green']['old_positive_to_zero'] and not by['green']['angle_change']
