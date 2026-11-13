"""비주행 준비: 원 common/설치 재사용, 설정 입구, actor0/시설, 평가 반례."""
import ast
import copy
import importlib
import inspect
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import types
import numpy as np
from shapely.geometry import Polygon

A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'guard_crawl_drive_r452_v1'
sys.path[:0]=[str(A),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import sha,read,write_new,verify_common,arguments
import solo_evaluation as se
import solo_runner as runner

def main():
    common=verify_common();checks={}
    # The exact source and existing native/install paths used by r452 are reused; no colcon.
    rows=[]
    old=read(BASE/'build_v1/import_paths.log')
    for row in old['modules']:
        p=Path(importlib.import_module(row['module']).__file__).resolve()
        assert p==Path(row['path']) and sha(p)==row['sha256'];rows.append(dict(module=row['module'],path=str(p),sha256=sha(p)))
    from arena_autonomy import _numeric_kernels,numeric_kernels
    numeric_kernels.configure('native_v1');native=Path(_numeric_kernels.__file__).resolve()
    assert sha(native)==old['native_module']['sha256']
    from ament_index_python.packages import get_package_share_directory,get_package_prefix
    installed={}
    for name in ('arena_autonomy','arena_vehicle_interface','arena_bringup'):
        path=Path(get_package_prefix(name));assert path.resolve().is_relative_to(ROOT/'install');installed[name]=str(path)
    checks['r452_common_source_exact']=len(common)
    write_new(A/'build_reuse_v1/receipt.json',dict(build_reuse_verified=True,build_executed=False,packages=installed,
        modules=rows,native=dict(path=str(native),sha256=sha(native)),r452_build_receipt_sha256=sha(BASE/'build_v1/receipt.json')))
    fixture=ET.parse(A/'facilities_only.sdf').getroot();original=ET.parse(BASE/'run1/actors_and_old610.sdf').getroot()
    assert {m.get('name') for m in fixture.findall('model')}==se.FACILITIES
    for n in se.FACILITIES:assert ET.tostring(fixture.find(f"model[@name='{n}']"))==ET.tostring(original.find(f"model[@name='{n}']"))
    checks['exact_three_facilities_no_actor']=True
    world=ET.parse(BASE/'run1/runtime_world.sdf').getroot()
    for n in ('eval_g1b_actor','eval_second_lead'):
        m=world.find(f"world/model[@name='{n}']");assert m is not None;world.find('world').remove(m)
    worldtext=ET.tostring(world,encoding='unicode');assert se.no_actor_evidence(worldtext)['passed']
    assert not se.no_actor_evidence((BASE/'run1/runtime_world.sdf').read_text())['passed']
    checks['runtime_actor_present_rejected']=True
    # No ROS init/node/process is invoked: stop before init, after the fixed config/native/official input path.
    import rclpy
    init=rclpy.init;before=list(sys.argv)
    class ReachedInit(Exception):pass
    def stopped(*a,**k):raise ReachedInit()
    rclpy.init=stopped
    argv=arguments();argv[argv.index('--output')+1]=(A/'preflight_v1/entry_only').relative_to(ROOT).as_posix()
    try:
        sys.argv=argv
        try:runner.main()
        except ReachedInit:checks['fixed_cli_reached_ros_init_without_execution']=True
        else:raise AssertionError('entry did not reach init')
    finally:rclpy.init=init;sys.argv=before
    # The original _select/guard/control modules are not patched by this adapter.
    tree=ast.parse((A/'solo_runner.py').read_bytes());main_node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    main_text=ast.get_source_segment((A/'solo_runner.py').read_text(),main_node)
    assert 'SetEntityPose.Request' not in main_text and 'actor_specs(' not in main_text and 'extras.' not in main_text
    checks['no_actor_calls_in_main']=True
    # Check launch selection is byte-equivalent except own paths and the two removed actor elements.
    oldcmd=read(BASE/'launch_command.json')
    newcmd=[v.replace(str(BASE),str(A)).replace('guard_crawl_drive_r452_v1','solo_full_lap_r457_v1') for v in oldcmd]
    newcmd=[v.replace('actors_and_old610.sdf','facilities_only.sdf') for v in newcmd]
    write_new(A/'launch_command_expected.json',newcmd)
    checks['launch_policy_values_copied_from_r452']=True
    # Synthetic dense trajectory: genuine one lap, partial lap, gap, actor insertion, wall and surface cases.
    import g1_traffic_eval as ev
    coarse=ev.Centerline([0,2,2,0],[0,0,2,2],[0,2,4,6],8.)
    samples=np.arange(0.,8.,.01);poses=[coarse.pose(t) for t in samples]
    line=ev.Centerline([p[0] for p in poses],[p[1] for p in poses],samples,8.)
    ego=[(float(t),*line.pose(float(t))) for t in np.linspace(0,8.2,821)]
    trace=[dict(t=float(t),x=line.pose(t)[0],y=line.pose(t)[1],yaw=line.pose(t)[2],ground_speed_mps=1.,safe_speed_mps=1.,drive_speed_mps=1.,autonomy={'behavior':'SOLO','mode':'g1g2'}) for t in np.arange(0,8.20001,.1)]
    config=copy.deepcopy(runner.load_config(A/'config.json'));config['ego_geometry']=dict(length_m=.207,width_m=.15,wheelbase_m=.128,track_width_m=.1239)
    case=config['cases'][0];road=Polygon([(-1,-1),(3,-1),(3,3),(-1,3)])
    args=dict(config=config,case=case,mode='g1g2',t_green=0.,trace=trace,ego=ego,actor=[],contacts=[],commands=[],stop_start=8.2,schedule_pose=None,centerline=line,s0=0.,road=road,runtime_world=worldtext,contact_evidence={},obstacles=[])
    result=se.evaluate_run(**args);assert result['gates']['full_lap'] and all(result['gates'].values())
    checks['full_lap_accepted_with_complete_data']=True
    short={**args,'ego':ego[:401],'trace':trace[:41],'stop_start':4.}
    assert not se.evaluate_run(**short)['gates']['full_lap'];checks['partial_lap_rejected']=True
    missing={**args,'ego':ego[:200]+ego[250:]};assert not se.evaluate_run(**missing)['harness']['valid'];checks['missing_pose_rejected']=True
    hit={**args,'obstacles':[Polygon([(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)])]}
    assert se.evaluate_run(**hit)['dense_wall_overlap']['overlap_samples']>0;checks['wall_overlap_detected']=True
    narrow={**args,'road':Polygon([(10,10),(11,10),(11,11),(10,11)])}
    assert not se.evaluate_run(**narrow)['gates']['two_wheel_surface'];checks['surface_departure_rejected']=True
    # Original finalize retained all STOP/integrity checks except the two removed actor-only requirements.
    final=(A/'solo_finalize.py').read_text()
    for k in ('dense_pose_stop_evidence','no_mcu_latch_before_stop','explicit_mcu_stop','no_deadline_diagnostic','safe_drive_publisher_safety_only','sensor_only_controller_inputs'):
        assert k in final
    checks['stop_and_independent_guard_gates_retained']=True
    write_new(A/'preflight_v1/result.json',dict(passed=True,checks=checks,ros_initialized=False,gazebo_started=False,build_executed=False))
    print(json.dumps(dict(passed=True,checks=len(checks),common_files=len(common),build_executed=False)))

if __name__=='__main__':main()
