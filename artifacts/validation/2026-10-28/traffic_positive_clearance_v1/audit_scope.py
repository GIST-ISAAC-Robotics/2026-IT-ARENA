"""저장 출력에서 거절 사유를 집계하고 대표 센서/광선만 재구성한다."""
import hashlib,json,math
from collections import Counter
from pathlib import Path
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene

BASE=Path(__file__).resolve().parent;path=BASE/'run1_course/follow_decisions.jsonl'
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
before=sha(path);counts=Counter();rows=[];targets=[130.82,130.84,131.5,135.]
with path.open(encoding='utf-8') as f:
    header=json.loads(next(f))['data'];motion=MotionConfig(**header['motion_config'])
    for line in f:
        r=json.loads(line)
        if r['kind']!='call':continue
        meta=r['data']['output'][2];same=meta.get('same_path_gap') or {}
        if same:counts[same.get('reason')]+=1
        t=r['data']['input']['reference_s']
        if not any(abs(t-v)<1e-8 for v in targets):continue
        data,raw,h=restore_call(r['data']['input'],motion)
        scene=motion_scene(raw,h,t,timing_verified=True,rear_half_angle_deg=data['rear_half_angle_deg'])
        proof=meta['partial_observation']['timed_missing_diagnostic']
        unknown=[]
        for ray in scene.rays:
            if ray.status!='unknown':continue
            old=(raw.angle_min+ray.source_index*raw.angle_increment+motion.lidar_yaw_rad+math.pi)%math.tau-math.pi
            now=math.atan2(ray.direction_xy[1],ray.direction_xy[0])
            unknown.append(dict(index=ray.source_index,at_acquisition_deg=math.degrees(old),now_deg=math.degrees(now),
                in_current_policy=abs(old)<=math.radians(100),in_same_path_scope=math.radians(60)<=abs(now)<=math.radians(100)))
        rows.append(dict(t_s=t,reason=same.get('reason'),proof=proof,unknown=unknown,
            original_selector_region=meta.get('selector_region')))
report=dict(scope='raw recorded reason and reconstructed ray directions only; no changed acceptance',
    input_sha256=before,input_unchanged=before==sha(path),reasons=dict(counts),examples=rows)
with (BASE/'run1_scope_audit.json').open('x',encoding='utf-8') as f:json.dump(freeze(report),f,ensure_ascii=False,indent=2)
print(json.dumps(report))
