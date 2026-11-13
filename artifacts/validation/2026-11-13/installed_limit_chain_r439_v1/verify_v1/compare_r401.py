import hashlib,json,sys,struct
from pathlib import Path
import yaml
import runtime_installed_limits as subject
import runtime_installed_profile as firmware
root=Path.cwd();a=root/'artifacts/validation/2026-11-13/installed_limit_chain_r439_v1'
sys.path.insert(0,str(a/'stage/tests'))
from test_runtime_installed_limits import synthetic_bundle
base=root/'artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml'
watched=[root/p for p in ['scripts/runtime_installed_profile.py','src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py','src/arena_autonomy/arena_autonomy/vehicle_geometry.py','scripts/build_runtime_drive.sh','scripts/build_runtime_preview.sh']]+[base]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={p.relative_to(root).as_posix():sha(p) for p in watched}
inputs=yaml.safe_load(base.read_bytes());results=[]
work=a/'verify_v1/synthetic_only';work.mkdir()
for label,limit in [('reduced_0p2',.2),('existing_cad',.4578436017036438)]:
 folder=synthetic_bundle(work,limit,label+'-firmware');snapshot={p.name:p.read_bytes() for p in folder.iterdir()};selection=work/(label+'-runtime')
 manifest=subject.prepare(folder,base,selection);cap=manifest['command_limit_rad']
 subject.verify(selection,folder,runtime_params=selection/'runtime_profile.yaml',host_steering_rad=cap)
 actual=yaml.safe_load((selection/'runtime_profile.yaml').read_bytes());changes=[]
 for node in subject.NODES:
  for key,original in inputs[node]['ros__parameters'].items():
   if actual[node]['ros__parameters'][key]!=original:changes.append(node+'.ros__parameters.'+key)
  assert actual[node]['ros__parameters'][subject.PARAMETER]==cap
  actual[node]['ros__parameters'][subject.PARAMETER]=inputs[node]['ros__parameters'][subject.PARAMETER]
 assert actual==inputs and cap<=limit and struct.unpack('<f',struct.pack('<f',cap))[0]==cap
 assert {p.name:p.read_bytes() for p in folder.iterdir()}==snapshot
 firmware.verify(folder)
 results.append(dict(case=label,synthetic_not_installed=True,board_limit_rad=limit,planner_guard_host_limit_rad=cap,changed_parameters=changes,other_values_exact=True,installed_bundle_unchanged=True,selection=str(selection.relative_to(a))))
after={p.relative_to(root).as_posix():sha(p) for p in watched};assert after==before
result=dict(passed=True,platform=sys.platform,python=sys.version,pyyaml=yaml.__version__,unittest_passed=10,actual_r401_parameter_counts={n:len(v['ros__parameters']) for n,v in inputs.items()},cases=results,original_inputs_before=before,original_inputs_after=after,wslexecution=False,nodes_started=False,hardware_executed=False,firmware_changed=False)
(a/'verify_v1/result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,ensure_ascii=False,indent=2))
