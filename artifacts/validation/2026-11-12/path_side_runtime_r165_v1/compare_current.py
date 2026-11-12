"""새 양수 명령의 현재 재검사. +20/+40 ms는 저장 입력 외삽의 제한 대조다."""
import copy
import importlib.util
import json
from pathlib import Path
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('r165_current_rebase', A/'rebase.py')
rebase = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rebase)
trace = A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'
events = [json.loads(line) for line in trace.open()]
header = events[0]['data']; cfg = FollowConfig(**header['follow_config'])
motion = MotionConfig(**header['motion_config'])
outputs = {r['seq']:r['output'] for r in map(json.loads,(A.parent/'path_side_gap_r164_v1/expected.jsonl').open())}
rows = []
for event in events[1:-1]:
    if event['kind'] != 'call':
        continue
    out = outputs[event['seq']]
    if not event['data']['output'][0] <= 0 < out[0]:
        continue
    assert rebase.supported(out)
    for offset in (0., .02, .04):
        submitted = event['data']['input']
        current = copy.deepcopy(submitted); current['reference_s'] += offset
        data, scan, history = restore_call(current, motion)
        try:
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            result = rebase.rebase(out, submitted, current, scene, history, cfg)
        except MotionUnavailable as error:
            # 실제 실행 연결에서도 같은 입력 실패는 0명령이다. 시각/이력을 연장하지 않는다.
            result = dict(speed_mps=0., reason='input_'+str(error), reached_rebase=False)
        rows.append(dict(seq=event['seq'], offset_s=offset, output=result))
summary = dict(tests=len(rows), positive=sum(r['output']['speed_mps']>0 for r in rows),
    reasons={reason:sum(r['output']['reason']==reason for r in rows)
             for reason in sorted(set(r['output']['reason'] for r in rows))},
    scope='same source scan and recorded motion history, reference time shift only; not new physical observations')
(A/'current_comparison.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2))
print(json.dumps(summary))
assert all(r['output']['speed_mps'] <= outputs[r['seq']][0] for r in rows)
