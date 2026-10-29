"""기록된 유한 재관측 이동으로 옛 위치를 현재 반환에 대조한다. 미연결 기하 비교."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.anchor_clearance_trial import fresh_clearance


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


if __name__ == '__main__':
    source = BASE/'run2/follow_decisions.jsonl'
    before = sha(source)
    rows=[]
    with source.open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data']
        config=MotionConfig(**header['motion_config'])
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data']; now=data['input']['reference_s']
            if now>135.:break
            if now<134.:continue
            partial=data['output'][2].get('partial_observation') or {}
            anchor=partial.get('anchor_diagnostic') or {}
            if not (anchor.get('fit') or {}).get('accepted'):continue
            inputs,scan,history=restore_call(data['input'],config)
            scene=motion_scene(scan,history,now,timing_verified=inputs['timing_verified'],
                rear_half_angle_deg=0.,epoch=inputs['epoch'])
            first=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            points,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,
                scan.range_min,scan.range_max,first,scan.time_increment,now,
                mode='both',timing_verified=True)
            # 기록된 같은 시각의 전체 retained set을 사용한다. 반환을 보간하지 않는다.
            corrected=np.asarray(thaw(anchor['corrected_points']))
            assert len(corrected)==anchor['threat_set']['unique_points']
            result=fresh_clearance(corrected,points,now,scene,history,points,ids)
            rows.append(dict(t=now,source_first_s=first,recorded_anchor_reason=anchor['reason'],
                recorded_clearance_reason=(partial.get('clearance_diagnostic') or {}).get('reason'),
                corrected_points=corrected.tolist(),candidate=result))
    result=dict(scope='saved corrected retained coordinates and current raw rays; no controller connection or new driving',
        input_sha256=before,input_unchanged=sha(source)==before,script_sha256=sha(Path(__file__)),
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in (
            ROOT/'src/arena_autonomy/arena_autonomy/anchor_clearance_trial.py',
            ROOT/'src/arena_autonomy/arena_autonomy/surface_anchor_trial.py',
            ROOT/'src/arena_autonomy/arena_autonomy/observation_support.py')},rows=rows)
    with (BASE/'run2_rolling_clearance_probe.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps([dict(t=r['t'],reason=r['candidate']['reason'],accepted=r['candidate']['accepted'],
        proofs=r['candidate'].get('proofs')) for r in rows]))
