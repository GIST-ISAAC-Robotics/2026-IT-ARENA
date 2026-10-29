"""정리한 진단 객체를 같은 저장 입력에 대조한다. 주행 명령은 만들지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from probe_surface_anchor import SurfaceAnchorProbe

def main():
    output=BASE/'object_probe.json'
    if output.exists():raise FileExistsError(output)
    evidence_path=BASE.parent/'timed_latch_v1/policy_replay_v1/report.json'
    e=json.loads(evidence_path.read_text())['first_latch']['latch_evidence']
    if e['pairs']!=len(e['witnesses']):raise ValueError('truncated evidence cannot start recovery probe')
    points=np.asarray([w['previous_point_now_xy'] for w in e['witnesses']])
    trace=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    rows=[];probe=SurfaceAnchorProbe();started=False;first=None
    with trace.open() as stream:
        motion=MotionConfig(**json.loads(next(stream))['data']['motion_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']!='call':continue
            inp=row['data']['input'];now=inp['reference_s']
            if now<e['reference_s']:continue
            if now>e['reference_s']+1.1:break
            d,scan,h=restore_call(inp,motion);source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            scene=motion_scene(scan,h,now,timing_verified=d['timing_verified'],epoch=d['epoch'],
                rear_half_angle_deg=d['rear_half_angle_deg'])
            rays=[r for r in scene.rays if r.status=='return']
            current=np.asarray([r.hit_xy_m for r in rays]);ids=np.array([r.source_index for r in rays])
            if not started:
                probe.start(points,current,ids,now,source);started=True;continue
            result=probe.observe(current,ids,h,now,source)
            if result['matched'] and first is None:first=now
            rows.append(dict(reference_s=now,source_s=source,**result))
    files=[Path(__file__),ROOT/'scripts/probe_surface_anchor.py',ROOT/'tests/test_surface_anchor_probe.py',
           *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    inputs=[evidence_path,trace]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files+inputs}
    with zipfile.ZipFile(BASE/'object_probe_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(p,p.relative_to(ROOT).as_posix())
    result=dict(scope='reobservation diagnostic only, original latch/commands remain unchanged',
        first_match_s=first,calls=len(rows),samples=rows,source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in hashes.items()))
    output.write_text(json.dumps(freeze(result),indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','source_sha256')}))

if __name__=='__main__':main()
