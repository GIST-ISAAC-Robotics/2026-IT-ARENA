"""r126 실제 현재 재계산 입력의 CPU 비용. ROS/센서/구동 접근 없음."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path.insert(0,str(A.parent/'follow_rebase_r126_v1'))
from rebase import rebase
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call,snapshot_call,freeze,equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionUnavailable


def summary(values):
    values=sorted(values)
    return dict(count=len(values),p50=values[len(values)//2],
        p95=values[int(.95*len(values))],maximum=values[-1],over_20ms=sum(v>20 for v in values))


def main():
    output=Path(sys.argv[1]);output.mkdir(exist_ok=False,parents=True)
    manifest=json.loads((A/'bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    def unchanged():
        return all(sha(ROOT/name)==row['sha256'] for name,row in manifest['files'].items())
    assert unchanged()
    trace=ROOT/manifest['trace']
    rows=[json.loads(line) for line in trace.open(encoding='utf8')]
    follow,motion=make_controller(rows[0]['data'])
    costs=defaultdict(list);mismatch=[];records=[]
    for row in rows:
        if row['kind']!='call' or 'current_rebase' not in row['data']:continue
        original=row['data'];saved=original['current_rebase']
        start=time.perf_counter()
        data,scan,history=restore_call(saved['input'],motion)
        costs['restore'].append(1000*(time.perf_counter()-start))
        start=time.perf_counter()
        current=snapshot_call(scan,history,data['reference_s'],data['epoch'],data['speed_mps'],
            data['side'],data['previous_steering_rad'],data['dt_s'],data['timing_verified'],
            data['rear_half_angle_deg'])
        snapshot_end=time.perf_counter()
        try:
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            scene_end=time.perf_counter()
            actual=rebase(original['output'],original['input'],current,scene,history,follow.config)
        except (MotionUnavailable,ValueError) as error:
            scene_end=time.perf_counter()
            actual=dict(speed_mps=0.,steering_rad=data['previous_steering_rad'],
                reference_s=data['reference_s'],reason='rebase_input_stop',input_error=str(error),
                independent_guard_required=True,runtime_authorized=False)
        actual=freeze(actual)
        end=time.perf_counter()
        costs['snapshot'].append(1000*(snapshot_end-start))
        costs['scene'].append(1000*(scene_end-snapshot_end))
        costs['rebase_and_freeze'].append(1000*(end-scene_end))
        costs['runtime_equivalent'].append(1000*(end-start))
        if not equal_output(current,saved['input']) or not equal_output(actual,saved['output']):
            mismatch.append(row['seq'])
        records.append(dict(seq=row['seq'],output=actual))
    assert records
    result=dict(calls=len(records),mismatch_sequences=mismatch,stages_ms={k:summary(v) for k,v in costs.items()},
        source_unchanged=unchanged(),python=platform.python_version(),numpy=np.__version__,
        machine=platform.machine(),trace_sha256=sha(trace),
        scope='Saved current snapshot/projection/rebase only; excludes planner, ROS, vision and independent guard; no real-time proof')
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'details.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in records))
    (output/'durations.json').write_text(json.dumps(costs))
    print(json.dumps(result))
    return 0 if not mismatch and result['source_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
