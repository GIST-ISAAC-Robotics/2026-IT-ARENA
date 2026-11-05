"""각본 실행 전에 공식 월드의 실제 도로·벽·첫 커브까지 남은 길이를 평가 전용으로 확인."""
from prepare import HERE,ROOT,save
from validate_g1_traffic import (load_config,load_centerline,case_named,actor_start_s,check_route,
                                 road_polygon,obstacle_polygons)

if __name__=='__main__':
    world=ROOT/'src/arena_gazebo/worlds/it_arena_official'
    cfg=load_config(HERE/'course.json')
    case=case_named(cfg,'straight_moving_lead')
    center,scene=load_centerline(world)
    start=actor_start_s(cfg,scene,0,case)
    result=check_route(cfg,center,case,start,road_polygon(world/'world.sdf'),obstacle_polygons(world/'world.sdf'))
    save(HERE/'course_geometry.json',result);print(result)
