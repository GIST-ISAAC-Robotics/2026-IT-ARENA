"""root/runner 전용 저장 입력 재생. 여기서는 WSL/ROS/장치를 실행하지 않는다.

python replay_native.py --mode original28
python replay_native.py --mode stage
각 프로세스는 해당 3파일만 읽기 overlay하여 같은 저장 6입력의 helper를 재계산한다.
"""
from pathlib import Path
import argparse, json, math, os, sys
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[key]='1'
sys.dont_write_bytecode=True
A=Path(__file__).resolve().parent; ROOT=A.parents[4]
parser=argparse.ArgumentParser(); parser.add_argument('--mode',choices=('original28','stage'),required=True)
parser.add_argument('--candidate-dir',choices=('stage','stage2'),default='stage2')
args=parser.parse_args()
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import arena_autonomy
selected=args.candidate_dir if args.mode=='stage' else 'original'
arena_autonomy.__path__.insert(0,str(A/selected/'src/arena_autonomy/arena_autonomy'))
import numpy as np
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig, anchor_error
from arena_autonomy import current_wall_reacquire as helper
from arena_autonomy.local_path import local_path
P=ROOT/'artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/stall_extract'
header=json.loads((P/'header.json').read_text())['record']['data']
config=FollowConfig(**header['follow_config']); motion=MotionConfig(**header['motion_config'])
rows=[]
for case in ('last_positive','first_zero_discarded','first_zero_candidate'):
    for phase in ('compute','current'):
        file=P/f'{case}_{phase}_input.json'
        if not file.exists(): continue
        data,scan,history=restore_call(json.loads(file.read_text()),motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],epoch=data['epoch'],
            rear_half_angle_deg=data['rear_half_angle_deg'],timing_policy=data.get('timing_policy','verified_only'))
        before=scene.occupied_endpoints_xy_m
        row=dict(case=case,phase=phase,reference_s=scene.t_ref_s)
        def traced(*a,**kw):
            if selected!='stage2': kw['minimum_ahead_m']=.28
            p,b=local_path(*a,**kw);row['boundary']=b
            return p,b
        helper.local_path=traced
        output=helper.current_wall_candidate(scene,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],config)
        row['output']=output
        assert before==scene.occupied_endpoints_xy_m
        bearings=[math.atan2(r.direction_xy[1],r.direction_xy[0]) for r in scene.rays if r.status=='return']
        opts=dict(bearings_rad=bearings) if args.mode=='stage' else {}
        path,info=local_path(np.asarray(before),'right',config.wall_offset_m,horizon=1.8,
            piecewise_corner=config.piecewise_wall_trial,minimum_radius=config.wheelbase_m/math.tan(config.max_steering_rad),
            narrow_occluder_bridge=config.narrow_occluder_bridge,**opts)
        row['normal_right_path_info']=info
        row['normal_right_anchor_error_m']=None if path is None else anchor_error(info,config)
        row['normal_right_anchor_limit_m']=config.boundary_anchor_error_m
        if args.mode=='stage':
            assert output is not None and output[0]>0, (case,phase,row)
            assert output[2]['current_wall_reacquire']['minimum_boundary_ahead_m']==.28
        rows.append(row)
output=dict(mode=args.mode,numpy_version=np.__version__,scope='same saved helper inputs; no closed-loop or full historical controller replay',rows=rows)
(A/f'native_{args.mode}.json').write_text(json.dumps(freeze(output),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(mode=args.mode,rows=[dict(case=r['case'],phase=r['phase'],
    ahead=r['boundary'].get('boundary_observed_ahead_m'),output=None if r['output'] is None else r['output'][:2],
    right_anchor=r['normal_right_anchor_error_m']) for r in rows]),indent=2))
