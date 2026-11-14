"""Windows의 저장 SDF 정적 검사. 기존 평가 순수 함수 본문을 사용하며 ROS/제어/주행은 실행하지 않는다."""
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
if os.name=='nt':sys.path.insert(0,str(Path(os.environ['LOCALAPPDATA'])/'Temp/arena-r520-python'))
sys.path[:0]=[str(A),str(ROOT/'scripts')]
import numpy as np
import shapely
from shapely.affinity import rotate,translate
from shapely.geometry import Polygon,LineString,box
from shapely.ops import unary_union
from field_wall_geometry import (SOURCE,DERIVED,SELECTED,Y0,Y1,THICKNESS,canonical,
    nominal_face_x,wave,read,sha,verify_derived,values,walls_element)


def exact_functions(path,names,namespace):
    source=ast.parse(path.read_bytes())
    nodes=[n for n in source.body if isinstance(n,ast.FunctionDef) and n.name in names]
    assert len(nodes)==len(names)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),namespace)


def save(path,data):
    with path.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2);f.write('\n')


def main():
    info=verify_derived()
    assert not (A/'geometry_right_side.json').exists()
    # heavy track-generation imports(cv2 등)를 피하고 실제 evaluator의 두 순수 함수만 그대로 로드한다.
    helper=types.ModuleType('build_experimental_track')
    helper.__dict__.update(math=math,ET=ET,Path=Path,Polygon=Polygon)
    exact_functions(ROOT/'scripts/build_experimental_track.py',{'rectangle','obstacle_polygons'},helper.__dict__)
    sys.modules['build_experimental_track']=helper
    rectangle,obstacle_polygons=helper.rectangle,helper.obstacle_polygons
    from audit_local_pursuit_road import road_polygon
    import g1_traffic_eval as ev
    from traffic_road_route import check_onroad_route
    from facility_geometry import load,slab_projection
    old_walls=walls_element(SOURCE);new_walls=walls_element(DERIVED)
    shapes={c.get('name')[:-4]:rectangle(*[values(c)[0][i] for i in (0,1,5)],*values(c)[1][:2]) for c in new_walls.findall('collision')}
    field_checks=[]
    actual_faces={}
    for wall in info['selected_walls']:
        name=wall['source_name'];original=old_walls.find(f"collision[@name='{name}_col']")
        visual=old_walls.find(f"visual[@name='{name}_vis']")
        panels=[]
        for i,limits in enumerate(info['continuous_panel_intervals_m']):
            parts=[shapes[g['name']] for g in wall['generated'] if g['panel']==i]
            union=unary_union(parts)
            assert union.geom_type=='Polygon' and union.is_valid and not union.interiors,(name,i,union.geom_type)
            panels.append(union)
        whole=unary_union([shapes[g['name']] for g in wall['generated']])
        gap_areas=[whole.intersection(box(0,lo,20,hi)).area for lo,hi in info['joint_gap_intervals_m']]
        assert max(gap_areas)<1e-10,(name,max(gap_areas))
        seam_distances=[p.distance(q) for p,q in zip(panels,panels[1:])]
        assert min(seam_distances)>=.020-1e-9
        boundary=[]
        for which,panel,old_piece in (('lower',panels[0],shapes[f'field_{name}_before']),('upper',panels[-1],shapes[f'field_{name}_after'])):
            joined=unary_union([panel,old_piece])
            # source yaw의 소수점 표기 때문에 경계에0.1um 이하 수치 차이가 있을 수 있다.
            assert panel.distance(old_piece)<1e-7,(name,which,panel.distance(old_piece))
            assert joined.buffer(1e-7).geom_type=='Polygon' and not joined.buffer(1e-7).interiors
            boundary.append(dict(side=which,physical_distance_m=panel.distance(old_piece),
                overlap_area_m2=panel.intersection(old_piece).area,connected_at_0p1um_tolerance=True,
                flat_connector_length_m=.001))
        for item in wall['generated']:
            c=new_walls.find(f"collision[@name='{item['name']}_col']")
            v=new_walls.find(f"visual[@name='{item['name']}_vis']")
            assert canonical(c.find('surface'))==canonical(original.find('surface'))
            assert canonical(v.find('material'))==canonical(visual.find('material'))
            assert values(c)[1][2]==values(original)[1][2]
        faces=[]
        for y in np.linspace(Y0,Y1,1701):
            if any(lo-1e-8<=y<=hi+1e-8 for lo,hi in info['joint_gap_intervals_m']):continue
            cut=whole.intersection(LineString([(8,float(y)),(12,float(y))]))
            assert not cut.is_empty,(name,float(y))
            inner=cut.bounds[0] if wall['outward_x_sign']==1 else cut.bounds[2]
            desired=nominal_face_x(original,wall['outward_x_sign'],float(y))+wave(float(y))
            faces.append((float(y),inner,desired))
        actual_faces[name]=faces
        field_checks.append(dict(wall=name,continuous_panels=len(panels),panel_connected=True,unintended_interior_holes=0,
            joint_count=len(seam_distances),minimum_actual_joint_distance_m=min(seam_distances),
            maximum_joint_slab_overlap_m2=max(gap_areas),boundary_connections=boundary,
            maximum_sampled_inner_face_error_from_sine_m=max(abs(x-expected) for _,x,expected in faces)))
    right=actual_faces['walls_main_left_5'];left=actual_faces['walls_main_left_25']
    assert [r[0] for r in right]==[r[0] for r in left]
    widths=[r[1]-l[1] for r,l in zip(right,left)]
    desired_widths=[r[2]-l[2] for r,l in zip(right,left)]
    assert max(abs(w-d) for w,d in zip(widths,desired_widths))<.001
    world=SOURCE.parent
    cfg=read(A/'config.json');case=cfg['cases'][0]
    space=dict(np=np,json=json,math=math,ev=ev,DEMO_WORLD=world)
    eval_path=ROOT/'scripts/validate_g1_traffic.py'
    exact_functions(eval_path,{'load_centerline','actor_start_s','check_straight','actor_pose_at'},space)
    centerline,scene=space['load_centerline'](world)
    s0=space['actor_start_s'](cfg,scene,0,case)
    road=road_polygon(DERIVED);old_road=road_polygon(SOURCE)
    assert road.symmetric_difference(old_road).area==0
    walls=obstacle_polygons(DERIVED);wall_union=unary_union(walls)
    route=check_onroad_route(cfg,centerline,case,s0,road,walls,space['check_straight'],space['actor_pose_at'])
    fixture=ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    fixture_cfg=read(fixture/'fixture.json');mesh=fixture/'support_frame_m.stl'
    assert sha(mesh)==fixture_cfg['mesh_sha256']
    _,_,triangles,_=load(mesh)
    actor_shape=slab_projection(triangles,cfg['actor']['bottom_m'],cfg['actor']['bottom_m']+cfg['actor']['height_m'])
    ego_shape=slab_projection(triangles,.0325,.0925)
    facilities=[translate(rotate(actor_shape,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in fixture_cfg['locations']]
    ego_facilities=[translate(rotate(ego_shape,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in fixture_cfg['locations']]
    travel=ev.actor_travel_m(case['schedule'],case['duration_s']);steps=math.ceil(travel/.01)
    actor_collisions=[];minimum_wall=math.inf;minimum_facility=math.inf;outside=0.
    for i in range(steps+1):
        pose=space['actor_pose_at'](centerline,s0+travel*i/steps,[],0.,case['actor_lateral_offset_m'])
        body=rectangle(*pose,cfg['actor']['length_m'],cfg['actor']['width_m'])
        minimum_wall=min(minimum_wall,body.distance(wall_union))
        outside=max(outside,body.difference(road).area)
        for j,shape in enumerate(facilities):
            minimum_facility=min(minimum_facility,body.distance(shape))
            if body.intersection(shape).area>1e-6:actor_collisions.append(dict(sample=i,facility=j))
    assert steps+1==601 and not actor_collisions
    slot=next(s for s in scene['starting_grid']['slots'] if s['index']==cfg['launch']['grid_slot'])
    old_report=read(A.parent/'current_guide_extent_drive_r515_v1/run1/report.json')
    geometry=old_report['vehicle_config']['ego_geometry']
    vehicle=ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
    assert sha(vehicle)==old_report['vehicle_config']['sha256']
    ego=rectangle(slot['x'],slot['y'],slot['yaw_rad'],geometry['length_m'],geometry['width_m'])
    initial_actor=rectangle(*space['actor_pose_at'](centerline,s0,[],0.,case['actor_lateral_offset_m']),cfg['actor']['length_m'],cfg['actor']['width_m'])
    ego_checks=dict(wall_overlap_m2=ego.intersection(wall_union).area,wall_gap_m=ego.distance(wall_union),
        facility_overlap_m2=max(ego.intersection(p).area for p in ego_facilities),
        facility_gap_m=min(ego.distance(p) for p in ego_facilities),road_outside_m2=ego.difference(road).area,
        actor_gap_m=ego.distance(initial_actor),actor_overlap_m2=ego.intersection(initial_actor).area)
    assert ego_checks['wall_overlap_m2']==0 and ego_checks['facility_overlap_m2']==0 and ego_checks['actor_overlap_m2']==0
    assert ego_checks['road_outside_m2']<1e-6 and ego_checks['actor_gap_m']>=.08
    geometry_result=dict(passed=True,original_case=case,route=route,actor_height_m=cfg['actor']['height_m'],
        actor_travel_m=travel,actor_path_samples=steps+1,maximum_sample_step_m=travel/steps,
        actor_minimum_derived_wall_gap_m=minimum_wall,actor_full_height_facility_gap_m=minimum_facility,
        facility_count=len(facilities),facility_collision_sample_count=0,collisions=[],actor_max_outside_body_area_m2=outside,
        initial_ego=ego_checks,current_ego_geometry=geometry,original_geometry_reused=False,
        derived_world_sha256=sha(DERIVED),source_world_sha256=sha(SOURCE),
        source_hashes={p.relative_to(ROOT).as_posix():sha(p) for p in (SOURCE,DERIVED,world/'scene.json',world/'centerline.csv',vehicle,mesh,
            eval_path,ROOT/'scripts/build_experimental_track.py',ROOT/'scripts/traffic_road_route.py')},
        scope='601 static actor poses and initial ego against the same derived walls; no closed-loop driving or hardware proof',
        platform=sys.platform,static_calculation_only=True,simulator_started=False)
    field_result=dict(passed=True,derived_sha256=sha(DERIVED),field_checks=field_checks,
        sampled_road_side_width_m=dict(minimum=min(widths),maximum=max(widths),nominal=min(desired_widths),
            maximum_chord_join_width_deviation_m=max(abs(w-d) for w,d in zip(widths,desired_widths))),
        same_phase_and_no_intentional_opposed_narrowing=True,original_track_surface_unchanged=True,
        dependency_versions=dict(numpy=np.__version__,shapely=shapely.__version__),
        windows_temp_dependency_path=str(Path(os.environ.get('LOCALAPPDATA',''))/'Temp/arena-r520-python') if os.name=='nt' else None,
        WSL_used=False,simulator_started=False,controller_executed=False)
    save(A/'geometry_right_side.json',geometry_result);save(A/'field_static_geometry.json',field_result)
    info.update(static_geometry_check_pending=False,static_geometry_receipt='field_static_geometry.json',
                actor_initial_ego_receipt='geometry_right_side.json',static_geometry_passed=True)
    (A/'field_geometry_audit.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(passed=True,actor_samples=steps+1,actor_minimum_wall_gap_m=minimum_wall,
        initial_ego=ego_checks,field_checks=field_checks,wall_width=field_result['sampled_road_side_width_m']),ensure_ascii=False))


if __name__=='__main__':main()
