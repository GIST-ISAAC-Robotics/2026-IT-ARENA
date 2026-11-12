"""r168 고정 입력 RELEASED 차로 유지 재검사. 실제 배송/주행을 대신하지 않는다."""
import collections
import copy
import json
from pathlib import Path
import statistics
import time
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
import current as candidate

A = Path(__file__).resolve().parent
TRACE = A.parent/'present_guard_runtime_r171_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    rows, times = [], []
    with TRACE.open() as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config'])
        cfg = FollowConfig(**header['follow_config'])
        for row in map(json.loads, stream):
            if row['kind'] != 'call' or not candidate.supported(row['data']['output']):
                continue
            item = row['data']; old = item['input']; output = item['output']
            for offset in (0., .02, .04):
                now = copy.deepcopy(old); now['reference_s'] += offset
                start = time.perf_counter()
                try:
                    _, scan, history = restore_call(now, motion)
                    scene = motion_scene(scan, history, now['reference_s'],
                        timing_verified=now['timing_verified'], epoch=now['epoch'],
                        rear_half_angle_deg=now['rear_half_angle_deg'])
                    result = candidate.rebase(output, old, now, scene, history, cfg)
                    assert result is not None and 0 <= result['speed_mps'] <= output[0]
                    assert result['path_source_s'] == output[2]['normal']['path_source_s']
                    rows.append(dict(seq=row['seq'], offset_s=offset, speed_mps=result['speed_mps'],
                        old_speed_mps=output[0], measured_speed_mps=now['speed_mps'], reason=result['reason'],
                        current_approach=result.get('current_approach'), region=result.get('region')))
                except MotionUnavailable as error:
                    rows.append(dict(seq=row['seq'], offset_s=offset, speed_mps=0., reason='input_'+str(error)))
                times.append(1000*(time.perf_counter()-start))
    counts = collections.Counter((r['offset_s'], r['reason'], r['speed_mps'] > 0) for r in rows)
    report = dict(input_calls=len(rows)//3, cases=len(rows),
        counts=[dict(offset_s=k[0], reason=k[1], positive=k[2], count=v) for k,v in counts.items()],
        cost_ms=dict(p50=statistics.median(times), p95=sorted(times)[int(.95*len(times))], max=max(times)),
        synthetic_same_scan_offsets=True, real_delivery_or_drive_test=False)
    (A/'r171_comparison_rows.json').write_text(json.dumps(rows, indent=2)+'\n')
    (A/'r171_comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
