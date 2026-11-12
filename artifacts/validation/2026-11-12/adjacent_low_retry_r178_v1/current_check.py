"""저속 재검사로 생긴 새 명령을 공통 현재 검사에 대조한다. 차량 발행 없음."""
import copy
import json
from collections import Counter
from pathlib import Path
from arena_autonomy import current_path_recheck as current_check
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

A = Path(__file__).resolve().parent
TRACE = A.parent/'released_parallel_runtime_r174_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    outputs = {row['seq']:row['output'] for row in map(json.loads, (A/'expected.jsonl').open())}
    rows = []
    with TRACE.open() as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config']); cfg = FollowConfig(**header['follow_config'])
        for event in map(json.loads, stream):
            if event['kind'] != 'call': continue
            item = event['data']; out = outputs[event['seq']]
            if not item['output'][0] <= 0 < out[0]: continue
            assert current_check.supported(out)
            for offset in (0., .02, .04):
                now = copy.deepcopy(item['input']); now['reference_s'] += offset
                try:
                    _, scan, history = restore_call(now, motion)
                    scene = motion_scene(scan, history, now['reference_s'], timing_verified=now['timing_verified'],
                                         epoch=now['epoch'], rear_half_angle_deg=now['rear_half_angle_deg'])
                    result = current_check.rebase(out, item['input'], now, scene, history, cfg)
                    assert result is not None and 0 <= result['speed_mps'] <= .15
                    assert result['path_source_s'] == out[2]['normal']['path_source_s']
                    rows.append(dict(seq=event['seq'], offset_s=offset, speed_mps=result['speed_mps'], reason=result['reason']))
                except MotionUnavailable as error:
                    rows.append(dict(seq=event['seq'], offset_s=offset, speed_mps=0., reason='input_'+str(error)))
    counts = Counter((r['offset_s'], r['reason'], r['speed_mps'] > 0) for r in rows)
    result = dict(calls=len(rows)//3, cases=len(rows), counts=[dict(offset_s=k[0], reason=k[1], positive=k[2], count=v) for k,v in counts.items()],
                  synthetic_same_scan_offsets=True, real_delivery_or_drive_test=False)
    (A/'current_check.json').write_text(json.dumps(result, indent=2)+'\n')
    (A/'current_rows.json').write_text(json.dumps(rows, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':main()
