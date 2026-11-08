"""완료한 r27 입력에 동일 P21 관측기를 적용. 지도/정답은 사후 표시에만 사용."""
import bisect
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PROBE = HERE.parent/'p21_sensor_preview_v1'
sys.path.insert(0, str(PROBE))
from observe import observe
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable

D = HERE.parent/'traffic_faster_reacquire_live_v1/two_actor_course_lap/run1'
source = D/'follow_decisions.jsonl'
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = digest(source)
rows = []
last = None
skipped = 0
with source.open() as stream:
    for line in stream:
        e = json.loads(line)
        if e['kind'] == 'header':
            mc = MotionConfig(**e['data']['motion_config'])
        if e['kind'] != 'call':
            continue
        d, raw, h = restore_call(e['data']['input'], mc)
        stamp = (raw.header.stamp.sec, raw.header.stamp.nanosec)
        if stamp == last:
            continue
        last = stamp
        try:
            scene = motion_scene(raw, h, d['reference_s'], timing_verified=d['timing_verified'],
                epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
        except (ValueError, MotionUnavailable):
            skipped += 1
            continue
        rows.append(dict(t=d['reference_s'], source=stamp, candidate=observe(scene)))

# 관측 생성이 끝난 뒤에만 평가 궤적·코스를 읽는다.
tr = json.loads((D/'trace.json').read_bytes())
times = [v['t'] for v in tr]
scene_path = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json'
scene = json.loads(scene_path.read_bytes())
s0 = next(b for b in scene['branches'] if 'P21' in b['name'])['s0_m']
lap = scene['track']['lap_length_m']
marker_times = [v['t'] for v in tr if 30 in v['autonomy'].get('marker_ids', [])]
for row in rows:
    i = bisect.bisect_left(times, row['t'])
    v = min(tr[max(0, i-1):min(len(tr), i+1)], key=lambda r: abs(r['t']-row['t']))
    distance = (s0-v['s']+lap/2) % lap-lap/2
    row.update(eval_s=v['s'], eval_entry_distance_m=distance, eval_near_entry=-.4<=distance<=2.2,
        eval_body_speed_mps=v['ground_speed_mps'], nearest_eval_dt=abs(v['t']-row['t']),
        prior_id30=bool(marker_times and marker_times[0]<=row['t']))
found = [r for r in rows if r['candidate']]
near = [r for r in found if r['eval_near_entry']]
result = dict(scope='same observer on recorded slower main-route traffic; no new steering or branch drive',
    scans=len(rows), skipped=skipped, found=len(found), near=len(near), far=len(found)-len(near),
    near_scans=sum(r['eval_near_entry'] for r in rows), near_candidates=near,
    marker30_first_state_s=min(marker_times, default=None), sensor_sha256=before,
    input_unchanged=before==digest(source), observer_sha256=digest(PROBE/'observe.py'),
    source_run=D.relative_to(ROOT).as_posix(), scene_sha256=digest(scene_path))
(HERE/'observations.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1)+'\n', encoding='utf-8')
(HERE/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False))
