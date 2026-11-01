"""10 km/h 명령의 필요 관측 거리와 실제로 같은 방향에 이어지는 반환 거리 대조(제어기 미연결)."""
import json
from collections import Counter
from pathlib import Path
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from straight_wall_end import examine

BASE=Path(__file__).resolve().parent


def compare(folder,window=None):
    states=Counter();rows=[];seen=set()
    with (folder/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        head=json.loads(next(stream))['data']
        motion=MotionConfig(**head['motion_config']);cfg=head['follow_config']
        for ev in map(json.loads,stream):
            if ev['kind']!='call':continue
            raw=ev['data']['input'];v,_,d=ev['data']['output'];source=d.get('source_first_s')
            if window is not None and not window[0]<=raw['reference_s']<=window[1]:continue
            if not d.get('open_route_speed_enabled') or v<2. or source in seen:continue
            seen.add(source);args,scan,hist=restore_call(raw,motion)
            scene=motion_scene(scan,hist,args['reference_s'],timing_verified=args['timing_verified'],
                               epoch=args['epoch'],rear_half_angle_deg=args['rear_half_angle_deg'])
            if scene.t_ref_s-scene.source_first_s<0:raise ValueError('negative scan age')
            age=scene.t_ref_s-scene.source_first_s
            goal=10/3.6
            # braking_speed(scope,age,2.0)의 역함수, 차체 전방 점유 판정에는 정지 여유도 추가.
            need=.06+goal*(age+.10)+goal**2/4
            a=examine(scene,d['path_wall'],cfg['wall_offset_m'])
            extra=0.;count=0;reason='no_extra_measured_face'
            if 'next_returns' in a:
                for p in a['next_returns']:
                    if (p['index_jump']!=1 or p['residual_from_prior_wall_m']>cfg['measurement_padding_m']
                            or p['along_prior_wall_m']<=extra):
                        break
                    extra=p['along_prior_wall_m'];count+=1
                # 한두 점은 진행 방향 근거로 쓰지 않는다. 점 사이를 자유 공간으로 판정하지 않는다.
                if count>=3:reason='three_collinear_extra_returns'
            bound=a.get('wall_ahead_m',0.)+(extra if count>=3 else 0.)
            states[reason]+=1
            rows.append(dict(time=scene.t_ref_s,command=v,extra_points=count,
                collinear_extra_m=extra,wall_bound_m=bound,path_need_m=need,
                occupancy_need_m=need+cfg['standstill_gap_m'],swept_clearance_m=d['swept_clearance_m'],
                path_m=d['observed_path_ahead_m'],
                wall_budget_reaches_goal=count>=3 and bound>=need))
    return dict(scans=len(rows),wall_support=dict(states),ray_geometry_can_cover_path_budget=sum(
        r['wall_budget_reaches_goal'] for r in rows),already_reaches_sweep_budget=sum(
        r['swept_clearance_m']>=r['occupancy_need_m'] for r in rows),samples=rows)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--lap-only',action='store_true')
    args=p.parse_args();window=None
    if args.lap_only:
        audit=json.loads((BASE/'solo_10kmh_metrics.json').read_text(encoding='utf-8'))
        start=audit['first_green_sample_s'];window=(start,start+audit['lap_s_from_first_green_sample'])
    result=compare(BASE/'solo_10kmh',window)
    result['lap_window_s']=window
    result['scope']='2 m/s 이상을 요청한 서로 다른 스캔의 첫 해당 판단. 연속 획득 광선과 3 cm 잔차는 후보 선별이지 미래 차체나 경로 안전의 보증이 아니다.'
    with (BASE/('straight_budget_lap.json' if window else 'straight_budget.json')).open('x',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2)
