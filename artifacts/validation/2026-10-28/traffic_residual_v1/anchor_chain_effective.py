"""센서 표면 정합을 순차 누적한 옛 점 대응 진단. 위치/속도 입력은 바꾸지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene,transport_points
from arena_autonomy.partial_surface_alignment import surface_translation
from arena_autonomy.partial_observation import short_observed_surface

def main():
    output=BASE/'anchor_chain_probe.json'
    if output.exists():raise FileExistsError(output)
    evidence_path=BASE.parent/'timed_latch_v1/policy_replay_v1/report.json'
    evidence=json.loads(evidence_path.read_text())['first_latch']['latch_evidence']
    trace=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    points=np.asarray([w['previous_point_now_xy'] for w in evidence['witnesses']])
    original_points=points.copy();last=None;rows=[];confirmed=0;first=None;previous=None;chain_valid=True
    with trace.open() as stream:
        motion=MotionConfig(**json.loads(next(stream))['data']['motion_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']!='call':continue
            inp=row['data']['input'];now=inp['reference_s']
            if now<evidence['reference_s']:continue
            if now>evidence['reference_s']+1.:break
            d,scan,h=restore_call(inp,motion)
            source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            if last==source:continue
            last=source
            scene=motion_scene(scan,h,now,timing_verified=d['timing_verified'],epoch=d['epoch'],
                rear_half_angle_deg=d['rear_half_angle_deg'])
            rays=[r for r in scene.rays if r.status=='return']
            current=np.asarray([r.hit_xy_m for r in rays]);ids=np.array([r.source_index for r in rays])
            fit=None
            if previous is not None:
                old,stamp=previous
                fit=surface_translation(transport_points(old,h,stamp,now),current,ids)
                candidate_points=transport_points(anchor_points,h,stamp,now)
                chain_valid=bool(fit['accepted'] and now-stamp<=.3)
                points=candidate_points + np.asarray(fit['translation_m']) if chain_valid else candidate_points
            else:
                # 초기점은 기존 래치 좌표다. 나중에 평가 정답으로 보정하지 않는다.
                assert abs(now-evidence['reference_s'])<1e-8
            nearest=np.min(np.linalg.norm(points[:,None]-current[None],axis=2),axis=1)
            observed=[bool(distance<=.02 or (distance<=.04 and short_observed_surface(p,current,ids) is not None))
                for p,distance in zip(points,nearest)]
            uncorrected=transport_points(original_points,h,evidence['reference_s'],now)
            correction=points-uncorrected
            confirmed=confirmed+1 if chain_valid and all(observed) else 0
            if confirmed>=2 and first is None:first=now
            rows.append(dict(reference_s=now,source_s=source,fit=fit,chain_valid=chain_valid,
                corrected_points=points.tolist(),correction_m=correction.tolist(),
                nearest_m=nearest.tolist(),observed=observed,consecutive_sources=confirmed))
            if previous is None or chain_valid:
                previous=(current,now)
                anchor_points=points.copy()
    inputs=[Path(__file__),evidence_path,trace,ROOT/'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py',
        ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    result=dict(scope='sensor-only accumulated surface association diagnostic; static-background assumption, no identity or safety proof',
        first_two_source_reobservation_s=first,samples=rows,source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items()))
    output.write_text(json.dumps(freeze(result),indent=2),encoding='utf-8')
    print(json.dumps(dict(first_two_source_reobservation_s=first,chain_valid=chain_valid,
        samples=[dict(t=r['reference_s'],observed=r['observed'],nearest_m=r['nearest_m'],
            correction_m=r['correction_m'],fit_reason=r['fit']['reason'] if r['fit'] else None) for r in rows])))

if __name__=='__main__':main()
