"""기존 복수 actor 경계로 3/5대 출발을 사전 등록한다. 실행하지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import validate_g1_traffic as validator
from traffic_actors import overlay_case, actor_specs, check_additional_routes
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons


def schedule(delay):
    return [dict(from_s=0.,to_s=delay,speed_mps=0.),dict(from_s=delay,to_s=12.,speed_mps=.35)]


def main():
    line,scene=validator.load_centerline(BASE/'fixture')
    slots={s['index']:s for s in scene['starting_grid']['slots']}
    road=road_polygon(BASE/'fixture/world.sdf');obstacles=obstacle_polygons(BASE/'fixture/world.sdf')
    for count,indices in ((3,(0,4)),(5,(0,1,2,4))):
        config_path=BASE/f'dense{count}_config.json';overlay_path=BASE/f'dense{count}_actors.json'
        audit_path=BASE/f'dense{count}_preflight.json'
        if any(p.exists() for p in (config_path,overlay_path,audit_path)):
            raise FileExistsError('fresh dense configuration required')
        config=json.loads((BASE/'two_car_config.json').read_text(encoding='utf-8'))
        config['scope']=f'사전 등록 {count}대 출발: 슬롯3 자차,25cm 각본 상자'
        config['launch']['entry_g1']='run_dense.py explicit launch overlay'
        config['launch']['temporary_traffic_light_trigger']='official_4red_candidate RGB; evaluation material state not controller input'
        config['matrix']={'g1':{'cases':['slow_lead'],'seeds':[0]},'pilot':f'dense{count} preregistered single run'}
        delays={i:(1.1 if i==0 else .8 if i==1 else .5) if count==5 else .5 for i in indices}
        config['cases'][0]['schedule']=schedule(delays[0])
        config['cases'][0]['expected']=f'{count}대 제한 출발: 진행/접촉/노면/STOP을 원래 교통 판정과 구별'
        additions=[];poses=[]
        for i in indices:
            slot=slots[i];cx,cy,yaw=line.pose(slot['s_m'])
            lateral=-(slot['x']-cx)*math.sin(yaw)+(slot['y']-cy)*math.cos(yaw)
            actual=validator.actor_pose_at(line,slot['s_m'],[],0.,lateral)
            poses.append(dict(slot=i,pose=list(actual),position_error_m=math.hypot(actual[0]-slot['x'],actual[1]-slot['y']),
                              yaw_error_rad=actual[2]-slot['yaw_rad']))
            if i==0:continue
            additions.append(dict(model_name=f'eval_m5_slot{i}',role='lead' if i==4 else 'side',
                start_center_gap_m=slot['s_m']-slots[3]['s_m'],lateral_offset_m=lateral,
                route_kind='side_road_straight',minimum_body_gap_m=.15,
                gap_basis='과거150mm 비교선 보존; 별도 M5에서는 실제 모든 쌍 간격/접촉/진행을 보고한다',schedule=schedule(delays[i])))
        overlay=dict(schema_version=1,registered_at='2026-10-27 before dense run',base_case='slow_lead',
                     scope=config['scope'],additional_actors=additions)
        case={**config['cases'][0],'additional_actors':additions}
        specs=actor_specs(config,case)
        s0=validator.actor_start_s(config,scene,0,case)
        primary_route=validator.check_route(config,line,case,s0,road,obstacles)
        starts,routes=check_additional_routes(config,case,specs,scene,0,line,road,obstacles,
            validator.actor_start_s,validator.check_route,validator.actor_pose_at)
        config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
        overlay_path.write_text(json.dumps(overlay,ensure_ascii=False,indent=2),encoding='utf-8')
        verified=validator.load_config(config_path)
        assert len(actor_specs(verified,overlay_case(verified['cases'][0],overlay_path)))==count-1
        paths=[Path(__file__),BASE/'two_car_config.json',config_path,overlay_path,BASE/'fixture/scene.json']
        audit=dict(car_count=count,poses=poses,primary_route=primary_route,additional_routes=routes,
                   sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
        audit_path.write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(audit,ensure_ascii=False))


if __name__=='__main__':main()
