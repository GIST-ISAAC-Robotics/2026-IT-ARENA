"""Windows 표준 라이브러리만 쓰는 정적 확인. ROS import 없음."""
import ast
import json
import xml.etree.ElementTree as ET
from prepare import A,BASE,ROOT,verify_common,write_new,sha,arguments
common=verify_common()
for p in [*A.glob('*.py'),*(A/'launch').glob('*.py'),*(A/'execution_v1').glob('*.py')]:ast.parse(p.read_bytes())
old=ET.parse(BASE/'run1/actors_and_old610.sdf').getroot();new=ET.parse(A/'facilities_only.sdf').getroot()
removed={m.get('name') for m in old.findall('model')}-{m.get('name') for m in new.findall('model')}
assert removed=={'eval_g1b_actor','eval_second_lead'}
for m in new.findall('model'):assert ET.tostring(m)==ET.tostring(old.find(f"model[@name='{m.get('name')}']"))
assert [ET.tostring(p) for p in old.findall('plugin')]==[ET.tostring(p) for p in new.findall('plugin')]
source=(A/'solo_runner.py').read_text();tree=ast.parse(source);main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main');body=ast.get_source_segment(source,main)
assert all(s not in body for s in ('extras.','actor_specs(','SetEntityPose.Request','actor[-1]','actor_client'))
assert '--actor-overlay' not in arguments()
old_config=json.loads((BASE/'config.json').read_bytes());new_config=json.loads((A/'config.json').read_bytes())
launch=dict(new_config['launch']);launch['entry_g1']=old_config['launch']['entry_g1'];assert launch==old_config['launch']
assert new_config['timeouts']==old_config['timeouts']
result=dict(passed=True,python_ast_valid=True,common_files=len(common),common_delta=[],removed_models=sorted(removed),
    facility_elements_exact=True,plugin_elements_exact=True,launch_values_unchanged=True,timeouts_unchanged=True,
    actual_execution_tested=False,build_executed=False)
write_new(A/'preflight_v1/windows_static.json',result);print(json.dumps(result))
