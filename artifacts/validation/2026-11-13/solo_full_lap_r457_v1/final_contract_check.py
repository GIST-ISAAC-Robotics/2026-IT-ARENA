"""최종 actor0 finalize의 원 STOP/벽/센서 경계를 Windows에서 검사."""
import ast
import copy
import json
from pathlib import Path
import zipfile
A=Path(__file__).resolve().parent;BASE=A.parent/'guard_crawl_drive_r452_v1'
from solo_finalize import finalize_verdict
with zipfile.ZipFile(BASE/'sources.zip') as z:source=z.read('scripts/g1_traffic_eval.py').decode()
node=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='finalize_verdict')
old='\n'.join(source.splitlines()[node.lineno-1:node.end_lineno])+'\n'
expected=old.replace("'contact_source_instrumented': report.get('contact_evidence', {}).get('validated_source') is True,", "'no_traffic_actor_in_world': evaluation.get('no_actor', {}).get('passed') is True,")
expected=expected.replace("'actor_pose_only_evaluator': report.get('actor_pose_subscribers') == ['g1b_traffic_validator'],", "'no_actor_pose_publishers': report.get('actor_pose_publishers') == {'eval_g1b_actor': [], 'eval_second_lead': []},")
assert expected==(A/'solo_finalize.py').read_text()
base=dict(evaluation=dict(harness={'valid':True},no_actor={'passed':True},controller_mode={'behavior_rows':1,'g1g2_rows':1},gates={'full_lap':True}),
    evaluation_readers={'valid':True},sensor_only_inputs=True,safety_sensor_only_inputs=True,
    actuation=dict(sensor_only_inputs=True,stop_latch_timing={'valid':True},latched_before_stop_samples=0),
    actor_pose_publishers={'eval_g1b_actor':[],'eval_second_lead':[]},drive_publishers=['local_pursuit'],safe_drive_publishers=['lidar_safety'],
    sequential_acquisition_verified=True,lidar_nominal_interface_passed=True,source_unchanged=True,
    signal_states=['red','yellow','green'],false_start=False,
    stop=dict(stable=True,pose_evidence={'passed':True},wall_collision_samples=0),actuation_stop_passed=True,
    actuation_diagnostics={'control_deadline_samples':0},wall_collision_samples=0,shutdown_clean=True)
assert finalize_verdict(copy.deepcopy(base),'g1g2')['passed']
cases=[]
for key,value in [('actuation_stop_passed',False),('wall_collision_samples',1),('sensor_only_inputs',False),
                  ('source_unchanged',False),('shutdown_clean',False)]:
    row=copy.deepcopy(base);row[key]=value;assert not finalize_verdict(row,'g1g2')['passed'];cases.append(key)
row=copy.deepcopy(base);row['actor_pose_publishers']['eval_g1b_actor']=['unexpected'];assert not finalize_verdict(row,'g1g2')['passed'];cases.append('unexpected_actor_publisher')
row=copy.deepcopy(base);row['evaluation']['no_actor']['passed']=False;assert not finalize_verdict(row,'g1g2')['passed'];cases.append('actor_in_world')
value=dict(passed=True,original_finalize_exact_except_two_actor_keys=True,positive_case=1,rejected_cases=cases,
    dense_wall_metric_diagnostic_only=True,original_wall_stop_gate_retained=True,
    note='Post-review removed new dense-wall acceptance gate to retain r452 criteria. Existing dense metric unchanged; seven helper results remain raw pre-review evidence.')
p=A/'preflight_v2/windows_final_contract.json'
with p.open('x',encoding='utf8') as f:json.dump(value,f,indent=2);f.write('\n')
print(json.dumps(value))
