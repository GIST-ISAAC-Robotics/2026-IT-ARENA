"""root가 NumPy2D cross 지원 환경에서 실행할 두 입력 비교. ROS/장치 접근 없음."""
from pathlib import Path
import copy,hashlib,json,math,os,sys
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
A=Path(__file__).resolve().parent;ROOT=A.parents[4]
sys.dont_write_bytecode=True
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import numpy as np
# 실패를 None helper 결과로 오인하지 않도록 공통 계산 전 환경을 명시 검사.
# 호환 monkeypatch, 환경 설치, 공통 수정은 하지 않는다.
np.cross(np.array([1.,0.]),np.array([0.,1.]))
from arena_autonomy.local_follow import FollowConfig,path_agrees,path_agrees_arc
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_path import local_path
from arena_autonomy.current_wall_reacquire import current_wall_candidate
cfg=FollowConfig(**json.loads((A/'follow_config.json').read_text()))

def output_row(value):
    if value is None:return None
    speed,steer,detail=value;path=np.asarray(detail['path_points'])
    return dict(speed_mps=speed,steering_rad=steer,path_wall=detail['path_wall'],wall_fallback=detail['wall_fallback'],
        terminal_path_y_m=float(path[-1,1]),terminal_path_heading_rad=float(math.atan2(*(path[-1]-path[-3])[::-1])),
        swept_clearance_m=detail['swept_clearance_m'],path_points=detail['path_points'],diagnostic=detail['current_wall_reacquire'])

def main():
    destination = A/(sys.argv[1] if len(sys.argv) > 1 else 'native_result.json')
    assert not destination.exists(),'preserve each native result'
    results=[]
    for name in ('preferred_continuous','preferred_fragmented'):
        raw=json.loads((A/(name+'_scan.json')).read_text());raw.pop('null_range_meaning')
        raw['ranges']=[math.inf if v is None else v for v in raw['ranges']]
        scene=scene_from_scan(**raw);before=copy.deepcopy(scene)
        points=np.asarray(scene.occupied_endpoints_xy_m)
        paths={};boundaries={}
        for side in ('left','right'):
            path,info=local_path(points,side,cfg.wall_offset_m,horizon=cfg.boundary_min_ahead_m,
                piecewise_corner=cfg.piecewise_wall_trial,minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),
                narrow_occluder_bridge=cfg.narrow_occluder_bridge)
            paths[side]=path;boundaries[side]={k:v for k,v in info.items() if k!='path_points'}
        # 같은 scene/cfg에서 각 벽의 후보를 호출하고 평가에만 비교한다.
        preferred=current_wall_candidate(scene,0.,'left',0.,.1,cfg)
        opposite=current_wall_candidate(scene,0.,'right',0.,.1,cfg)
        both=all(v is not None for v in paths.values())
        assert scene==before and [r.source_index for r in scene.rays]==list(range(500))
        results.append(dict(case=name,boundaries=boundaries,preferred_policy_output=output_row(preferred),
            opposite_policy_output=output_row(opposite),raw_input_unchanged=True,
            path_agrees=bool(path_agrees(paths['left'],paths['right'],cfg.boundary_agreement_m)) if both else None,
            path_agrees_arc=bool(path_agrees_arc(paths['left'],paths['right'],cfg.boundary_agreement_m)) if both else None))
    expected=json.loads((A/'prepared_source_manifest.json').read_text())
    actual={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in expected}
    result=dict(cases=results,source_sha256=actual,source_matches_preparation=actual==expected,
        scope='two static synthetic sensor inputs only; no executed wrong turn or route completion claim',
        numpy=np.__version__,no_truth_passed_to_controller=True,ROS=False,Gazebo=False)
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for row in results:
        value=row['preferred_policy_output']
        print(json.dumps(dict(case=row['case'],preferred_boundary=row['boundaries']['left'].get('reason'),
            output=None if value is None else {k:v for k,v in value.items() if k not in ('path_points','diagnostic')},
            path_agrees=row['path_agrees'],path_agrees_arc=row['path_agrees_arc']),ensure_ascii=False))
if __name__=='__main__':main()
