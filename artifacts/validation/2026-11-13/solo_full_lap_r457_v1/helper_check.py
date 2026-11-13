"""첫 preflight의 남은 helper 검사만 재개. 원 실패는 보존한다."""
import ast
import copy
import json
from pathlib import Path
import sys
import numpy as np
from shapely.geometry import Polygon
import xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'guard_crawl_drive_r452_v1'
sys.path[:0]=[str(A),str(ROOT/'scripts')]
import g1_traffic_eval as ev
import solo_evaluation as se
from prepare import read,write_new,sha

coarse=ev.Centerline([0,2,2,0],[0,0,2,2],[0,2,4,6],8.)
samples=np.arange(0.,8.,.01);poses=[coarse.pose(t) for t in samples]
line=ev.Centerline([p[0] for p in poses],[p[1] for p in poses],samples,8.)
ego=[(float(t),*line.pose(float(t))) for t in np.linspace(0,8.2,821)]
trace=[dict(t=float(t),x=line.pose(t)[0],y=line.pose(t)[1],yaw=line.pose(t)[2],ground_speed_mps=1.,safe_speed_mps=1.,drive_speed_mps=1.,autonomy={'behavior':'SOLO','mode':'g1g2'}) for t in np.arange(0,8.20001,.1)]
world=ET.parse(BASE/'run1/runtime_world.sdf').getroot()
for n in ('eval_g1b_actor','eval_second_lead'):world.find('world').remove(world.find(f"world/model[@name='{n}']"))
worldtext=ET.tostring(world,encoding='unicode')
config=read(A/'config.json');config['ego_geometry']=dict(length_m=.207,width_m=.15,wheelbase_m=.128,track_width_m=.1239)
case=config['cases'][0];road=Polygon([(-1,-1),(3,-1),(3,3),(-1,3)])
args=dict(config=config,case=case,mode='g1g2',t_green=0.,trace=trace,ego=ego,actor=[],contacts=[],commands=[],stop_start=8.2,schedule_pose=None,centerline=line,s0=0.,road=road,runtime_world=worldtext,contact_evidence={},obstacles=[])
result=se.evaluate_run(**args)
assert result['gates']['full_lap'] and all(result['gates'].values()),result
checks={'full_lap_accepted_with_complete_data':True}
short={**args,'ego':ego[:401],'trace':trace[:41],'stop_start':4.}
assert not se.evaluate_run(**short)['gates']['full_lap'];checks['partial_lap_rejected']=True
missing={**args,'ego':ego[:200]+ego[250:]};assert not se.evaluate_run(**missing)['harness']['valid'];checks['missing_pose_rejected']=True
hit={**args,'obstacles':[Polygon([(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)])]}
assert se.evaluate_run(**hit)['dense_wall_overlap']['overlap_samples']>0;checks['wall_overlap_detected']=True
narrow={**args,'road':Polygon([(10,10),(11,10),(11,11),(10,11)])}
assert not se.evaluate_run(**narrow)['gates']['two_wheel_surface'];checks['surface_departure_rejected']=True
assert not se.no_actor_evidence((BASE/'run1/runtime_world.sdf').read_text())['passed'];checks['actor_world_rejected']=True
final=(A/'solo_finalize.py').read_text()
for k in ('dense_pose_stop_evidence','no_mcu_latch_before_stop','explicit_mcu_stop','no_deadline_diagnostic','safe_drive_publisher_safety_only','sensor_only_controller_inputs'):assert k in final
checks['stop_and_guard_gates_retained']=True
write_new(A/'preflight_v2/helper_result.json',dict(passed=True,checks=checks,complete_sample_result=result,
    first_failure='4-vertex synthetic centerline caused >1m projection jumps; no production evaluator condition changed',
    original_failure_log_sha256=sha(A/'preflight_v1/output.log')))
# Original preflight reached the first helper assertion after all prior checks and stopped without ROS init.
assert read(A/'build_reuse_v1/receipt.json')['build_reuse_verified']
assert (A/'preflight_v1/entry_only/facilities_only.sdf').read_bytes()==(A/'facilities_only.sdf').read_bytes()
log=(A/'preflight_v1/output.log').read_text();assert "result=se.evaluate_run(**args);assert" in log and 'AssertionError' in log
write_new(A/'preflight_v1/result.json',dict(passed=True,initial_preflight_exit=1,initial_failure_preserved=True,
    initial_checks_completed=['common1307 exact','native and installed package import reuse','exact three facility elements',
    'runtime actor present rejected','fixed CLI reached intercepted ROS init','main contains no actor command path',
    'launch expected arguments copied from r452'],helper_checks=checks,ros_initialized=False,gazebo_started=False,
    build_executed=False,helper_result='preflight_v2/helper_result.json'))
print(json.dumps(dict(passed=True,helper_checks=len(checks),ros_initialized=False,gazebo_started=False)))
