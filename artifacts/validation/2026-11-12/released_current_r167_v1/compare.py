"""r165 저장 입력의 RELEASED 양수 결과를 현재 좌표에서 다시 검사한다. 주행 아님."""
import collections
import copy
from dataclasses import replace
import json
from pathlib import Path
import time
import statistics

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
import released

A=Path(__file__).resolve().parent
TRACE=A.parent/'path_side_runtime_r165_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    result=[];times=[]
    with TRACE.open() as stream:
        header=json.loads(next(stream))['data']
        motion=MotionConfig(**header['motion_config']);cfg=FollowConfig(**header['follow_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']!='call':continue
            item=row['data'];output=item['output']
            if not released.supported(output):continue
            old=item['input']
            for offset in (0.,.02,.04):
                current=copy.deepcopy(old);current['reference_s']+=offset
                start=time.perf_counter()
                try:
                    values,scan,history=restore_call(current,motion)
                    scene=motion_scene(scan,history,current['reference_s'],timing_verified=current['timing_verified'],
                        epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
                    answer=released.rebase(output,old,current,scene,history,cfg)
                    assert answer is not None and 0<=answer['speed_mps']<=output[0]
                    assert answer['path_source_s']==output[2]['normal']['path_source_s']
                    result.append(dict(seq=row['seq'],offset_s=offset,speed_mps=answer['speed_mps'],
                        previous_speed_mps=output[0],reason=answer['reason'],reference_s=answer['reference_s']))
                except MotionUnavailable as e:
                    result.append(dict(seq=row['seq'],offset_s=offset,speed_mps=0.,reason='input_'+str(e)))
                times.append(1000*(time.perf_counter()-start))
    counts=collections.Counter((r['offset_s'],r['reason'],r['speed_mps']>0)for r in result)
    summary=dict(input_calls=len(result)//3,cases=len(result),
        counts=[dict(offset_s=k[0],reason=k[1],positive=k[2],count=v)for k,v in counts.items()],
        cost_ms=dict(p50=statistics.median(times),p95=sorted(times)[int(.95*len(times))],max=max(times)),
        current_sensor_frame_synthetic_offset=True,real_delivery_or_drive_test=False)
    (A/'comparison_rows.json').write_text(json.dumps(result,indent=2)+'\n')
    (A/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
