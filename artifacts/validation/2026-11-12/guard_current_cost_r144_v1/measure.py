"""동결된 현재 GUARD 계산의 CPU 비용만 측정한다. 센서/ROS/출력 접근 없음."""
import hashlib
import json
from pathlib import Path
import sys
import time

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
             str(A.parent/'guard_current_r142_v1')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, snapshot_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene
from candidate import rebase


def summary(values):
    values=sorted(values)
    return dict(count=len(values),p50=values[len(values)//2],p95=values[int(.95*len(values))],maximum=values[-1],
                over20=sum(v>20 for v in values),over100=sum(v>100 for v in values)) if values else dict(count=0)


def main():
    manifest=json.loads((A/'bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    unchanged=lambda:all(sha(ROOT/name)==value['sha256'] for name,value in manifest['files'].items())
    assert unchanged()
    rows=[json.loads(line) for line in (A/'samples.jsonl').open()]
    owner,motion=make_controller(rows[0]['header'])
    prepared=[(row,restore_call(row['current'],motion)) for row in rows[1:]]
    result=[];mismatches=[]
    for row,(data,scan,history) in prepared:
        began=time.perf_counter()
        snapshot=snapshot_call(scan,history,data['reference_s'],data['epoch'],data['speed_mps'],
            data['side'],data['previous_steering_rad'],data['dt_s'],data['timing_verified'],data['rear_half_angle_deg'])
        copied=time.perf_counter()
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
            epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        projected=time.perf_counter()
        answer=freeze(rebase(row['original']['output'],row['original']['input'],snapshot,scan,scene,history,owner.config))
        finished=time.perf_counter()
        exact=equal_output(answer,row['expected']);snapshot_exact=equal_output(snapshot,row['current'])
        if not exact or not snapshot_exact:mismatches.append(row['seq'])
        result.append(dict(seq=row['seq'],reason=answer['reason'],positive=answer['speed_mps']>0,
            total_ms=1000*(finished-began),snapshot_ms=1000*(copied-began),
            projection_ms=1000*(projected-copied),guard_ms=1000*(finished-projected),
            output_exact=exact,snapshot_exact=snapshot_exact))
    output=dict(calls=len(result),mismatches=mismatches,source_unchanged=unchanged(),
                total_ms=summary([r['total_ms'] for r in result]),
                positive_ms=summary([r['total_ms'] for r in result if r['positive']]),
                stages={key:summary([r[key] for r in result if r['positive']])
                        for key in ('snapshot_ms','projection_ms','guard_ms')},results=result,
                scope='One pass of frozen simulation inputs on CPU, no ROS concurrency or physical sensor/control proof.')
    with (A/'measurement.json').open('x') as stream:json.dump(output,stream,indent=2)
    print(json.dumps({k:v for k,v in output.items() if k!='results'}))
    assert not mismatches and output['source_unchanged']


if __name__=='__main__':main()
