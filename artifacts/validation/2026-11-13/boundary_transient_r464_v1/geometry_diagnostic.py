"""저장 scan/history의 순수 Python 경계 함수 진단. 전체 plan 재생 아님."""
from pathlib import Path
import json
import sys
import math

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE/'original/src/arena_autonomy'))
sys.path.insert(0, str(HERE/'original/src/arena_vehicle_interface'))
import numpy as np
# 이전 NumPy의 2D cross 식만 진단 프로세스에서 보존. frozen source는 수정하지 않는다.
# 설치된 NumPy 2.5.4에서 삭제된 API 호환이며 수치 허용치/판정선 변경이 아니다.
original_cross = np.cross
def diagnostic_cross(a, b, *args, **kwargs):
    aa, bb = np.asarray(a), np.asarray(b)
    if not args and not kwargs and aa.shape[-1] == bb.shape[-1] == 2:
        return aa[..., 0] * bb[..., 1] - aa[..., 1] * bb[..., 0]
    return original_cross(a, b, *args, **kwargs)
np.cross = diagnostic_cross
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, path_agrees, path_agrees_arc, anchor_error
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene, transport_points

def agreement(a, b, tol):
    return {'shared_x': bool(path_agrees(a, b, tol)),
            'shared_arc': bool(path_agrees_arc(a, b, tol))}

def distance_diagnostics(a, b):
    result={}
    for mode in ('shared_x','shared_arc'):
        paths,coords=[],[]
        for p in (a,b):
            if mode=='shared_arc':
                p=p[int(np.argmin(np.linalg.norm(p,axis=1))):]
                s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
                keep=(s>=.05)&(s<=1.)
            else:
                s=p[:,0];keep=(s>=.05)&(s<=.65)
            paths.append(p[keep]);coords.append(s[keep])
        lo=max(float(x.min()) for x in coords);hi=min(float(x.max()) for x in coords)
        selected=[p[(s>=lo-1e-12)&(s<=hi+1e-12)] for p,s in zip(paths,coords)]
        distances=np.linalg.norm(selected[0][:,None,:]-selected[1][None,:,:],axis=2)
        result[mode]={'overlap_m':hi-lo,'points':[len(p) for p in selected],
            'selected_x_span_m':[float(np.ptp(p[:,0])) for p in selected],
            'symmetric_p80_m':float(max(np.percentile(distances.min(axis=0),80),
                                      np.percentile(distances.min(axis=1),80)))}
    return result

def main():
    source = HERE.parent/'solo_full_lap_r457_v1/run1/follow_decisions.jsonl'
    with source.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
    cfg, motion = FollowConfig(**header['follow_config']), MotionConfig(**header['motion_config'])
    rows=[json.loads(x) for x in (HERE/'saved_plan_window.jsonl').read_text().splitlines()]
    inputs={r['data']['sequence']:r['data']['input'] for r in rows if r['kind']=='runtime_plan_submitted'}
    outputs={r['data']['sequence']:r['data']['output'][2]['normal'] for r in rows if r['kind']=='runtime_plan_completed'}
    reference=np.asarray(outputs[639]['path_points'])
    reference_time=outputs[639]['reference_stamp_s']
    reference_source=outputs[639]['path_source_s']
    all_paths={'recorded_639_path': reference.tolist()}
    results=[]
    for sequence in (639,641,643,644,645):
        data, scan, history=restore_call(inputs[sequence], motion)
        now=data['reference_s']
        scene=motion_scene(scan,history,now,timing_verified=data['timing_verified'],
                           epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
        moved=transport_points(reference,history,reference_time,now)
        summary={'plan_sequence':sequence, 'reference_s':now,
                 'recorded_reason':outputs[sequence]['reason'],
                 'original_memory_source_s':reference_source, 'memory_age_s':now-reference_source,
                 'max_reference_motion_m':float(np.linalg.norm(moved-reference,axis=1).max()),
                 'occupied_count':len(points), 'paths':{}}
        all_paths[str(sequence)]={'transported_639':moved.tolist()}
        for side in ('left','right'):
            path,detail=local_path(points,side,cfg.wall_offset_m,horizon=1.8,piecewise_corner=True,
                minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),narrow_occluder_bridge=True)
            sd={k:v for k,v in detail.items() if k!='path_points'}
            if path is not None:
                sd['anchor_error_m']=anchor_error(detail,cfg)
                sd['sufficient']=(detail['boundary_observed_ahead_m']>=cfg.boundary_min_ahead_m
                                  and anchor_error(detail,cfg)<=cfg.boundary_anchor_error_m)
                sd['versus_639']=agreement(path,moved,cfg.boundary_agreement_m)
                if side=='left': sd['agreement_diagnostic']=distance_diagnostics(path,moved)
                all_paths[str(sequence)][side]=path.tolist()
            summary['paths'][side]=sd
        if all(side in all_paths[str(sequence)] for side in ('left','right')):
            summary['bilateral']=agreement(np.asarray(all_paths[str(sequence)]['left']),
                                          np.asarray(all_paths[str(sequence)]['right']),cfg.boundary_agreement_m)
        results.append(summary)
    report={'scope':'Windows pure Python current-boundary geometry; no full Evaluator/native/drive replay',
            'numpy_version':np.__version__, 'numpy_2d_cross_compatibility':'a0*b1-a1*b0; diagnostic process only',
            'source_archive_sha256':json.loads((HERE/'original_manifest.json').read_text())['archive_sha256'],
            'config':{k:getattr(cfg,k) for k in ('traffic_boundary_refresh_trial','piecewise_wall_trial',
                'junction_continuity','expiry_route_reference_trial','memory_ttl_s','memory_motion_limit_m',
                'boundary_min_ahead_m','boundary_anchor_error_m','boundary_agreement_m','confirmations')},
            'rows':results}
    (HERE/'geometry_diagnostic.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (HERE/'geometry_paths.json').write_text(json.dumps(all_paths),encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
