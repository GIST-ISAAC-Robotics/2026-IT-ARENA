"""M5 2대 제한 출발 설정을 원래 교통 평가기와 대조해 사전 등록한다."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import validate_g1_traffic as validation
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons


def main():
    output=BASE/'two_car_config.json'
    if output.exists():raise FileExistsError(output)
    config=json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    line,scene=validation.load_centerline(BASE/'fixture')
    slots={s['index']:s for s in scene['starting_grid']['slots']}
    ego,peer=slots[3],slots[0]
    cx,cy,heading=line.pose(peer['s_m'])
    lateral=-(peer['x']-cx)*math.sin(heading)+(peer['y']-cy)*math.cos(heading)
    config['registered_at']='2026-10-27 before two-car physical trial'
    config['scope']='공식 신호 RGB·그리드3 자차/그리드0 근처 각본1대의12초 제한 출발, 실물/5대/완주 아님'
    config['launch'].update(grid_slot=3,speed_profile='g1_compare_1mps',lidar_acquisition='sequential',
        lidar_compensation='both',lidar_rate_hz=10,control_rate_hz=50,render_backend='wsl_nvidia',
        sensor_wall_timeout_s=30,speed_bump=False,actuation_mode='virtual_mcu')
    config['actor'].update(start_center_gap_m=peer['s_m']-ego['s_m'],lateral_offset_m=lateral,height_m=.25)
    config['actor']['start_center_gap_note']='공식 슬롯0/3의s차이, 아래actual pose 오차를 별도 등록'
    config['actor']['visible_section']='.095m C1면에 걸리는20x15x25cm 불투명 상자, 실물 외형 아님'
    config['cases']=[dict(name='slow_lead',route_kind='side_road_straight',duration_s=12.,
        schedule=[dict(from_s=0.,to_s=.5,speed_mps=0.),dict(from_s=.5,to_s=12.,speed_mps=.35)],
        expected='초록 후 앞옆차가 움직여 공간을 만들면 센서 기반 출발; 원래15cm 판정은 별도 보존')]
    config['seeds']={str(i):dict(actor_start_offset_m=0.) for i in range(3)}
    config['seed_note']='이 작업은seed0한 번, 다른seed실행 미등록'
    config['m5_acceptance']=dict(minimum_progress_m=1.,minimum_swept_gap_exclusive_m=0.,
        maximum_three_plus_wheel_samples=0,require_original_run_valid=True,require_all_stop_checks=True,
        note='기존150mm/final-gap/behind 등 판정은 삭제하지 않는다. 별도 M5관찰로 기록하며 실물 여유 보장 아님')
    case=config['cases'][0];s0=validation.actor_start_s(config,scene,0,case)
    actual=validation.actor_pose_at(line,s0,[],0.,lateral)
    route=validation.check_route(config,line,case,s0,road_polygon(BASE/'fixture/world.sdf'),obstacle_polygons(BASE/'fixture/world.sdf'))
    audit=dict(slot0_pose=[peer['x'],peer['y'],peer['yaw_rad']],actor_initial_pose=list(actual),
        position_error_m=math.hypot(actual[0]-peer['x'],actual[1]-peer['y']),yaw_error_rad=actual[2]-peer['yaw_rad'],
        ego_slot3=ego,route=route,
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                     (Path(__file__),ROOT/'config/tests/g1_traffic_v1.json',BASE/'fixture/world.sdf',BASE/'fixture/scene.json',BASE/'fixture/centerline.csv')})
    output.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
    (BASE/'two_car_preflight.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(audit,ensure_ascii=False))


if __name__=='__main__':main()
