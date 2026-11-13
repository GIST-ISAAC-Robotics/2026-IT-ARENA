from pathlib import Path
import ast,copy,hashlib,json,shutil,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=ROOT/'artifacts/validation/2026-11-13/estimated_observation_cost_r447_v1/bundle'
prior=ROOT/'artifacts/validation/2026-11-14/jetson_plan_cost_r482_v1'
B=A/'bundle';B.mkdir()
files={};provenance={}
def put(p,name):
    q=B/name;q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
    files[name]=sha(q);provenance[name]=dict(path=str(p.relative_to(ROOT)),sha256=sha(p))
for package in ('arena_autonomy','arena_vehicle_interface'):
    for p in sorted((ROOT/'src'/package/package).rglob('*')):
        if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h'):put(p,p.relative_to(ROOT).as_posix())
put(ROOT/'src/arena_autonomy/setup.py','src/arena_autonomy/setup.py')
put(old/'contract_reference.py','contract_reference.py')
put(A/'run_smoke.py','run_smoke.py')
put(prior/'environment_reference.py','environment_reference.py')
put(prior/'execute_profile_v2.py','ownership_reference.py')
all_rows=[json.loads(l) for l in (old/'inputs/selected.jsonl').read_text().splitlines()]
keys=[('r437',643),('r426',369),('r426',680),('r426',614)]
records=[next(r for r in all_rows if (r['dataset'],r['trace_seq'])==key) for key in keys]
headers={d:json.loads((old/'inputs'/f'{d}_header.json').read_bytes()) for d in ('r437','r426')}
data=dict(records=records,headers=headers)
(B/'inputs.json').write_text(json.dumps(data,allow_nan=False)+'\n',encoding='utf-8');files['inputs.json']=sha(B/'inputs.json')
# 기존 변환 원문을 추출해 계약 필드만 바뀌며 원 수치가 유지되는지 확인한다.
tree=ast.parse((old/'contract_reference.py').read_text(encoding='utf-8'))
func=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='estimated_record')
namespace=dict(copy=copy,ESTIMATED_SCOPE='host_estimated_crop_v1',component_scope=lambda x:'contract_marker')
exec(compile(ast.Module(body=[func],type_ignores=[]),'original_contract','exec'),namespace)
checks=[]
for r in records:
    converted=namespace['estimated_record'](r)
    for which in ('plan_input','current'):
        for k in ('timing_verified','timing_policy'):
            if k in r[which]:converted[which][k]=r[which][k]
            else:converted[which].pop(k,None)
        converted[which]['scan']['measurements']['time_basis']=r[which]['scan']['measurements']['time_basis']
    c=converted['plan_output'][2].get('normal',{}).get('partial_observation',{}).get('crop_edge_component')
    if c is not None:c['scope']=r['plan_output'][2]['normal']['partial_observation']['crop_edge_component']['scope']
    assert converted==r
    checks.append(dict(dataset=r['dataset'],trace_seq=r['trace_seq'],raw_numeric_preserved=True))
reuse=json.loads((prior/'profile_manifest_v2.json').read_bytes())['native_reuse']
for name,h in reuse['source_sha256'].items():assert sha(B/name)==h,name
source=(prior/'execute_profile_v2.py').read_text(encoding='utf-8')
source=source.replace('it-arena-r482-a','it-arena-r487-a').replace('profile_manifest_v2.json','manifest.json')
source=source.replace("A/'base'/p","A/p").replace("A/'base/src/","A/'src/")
source=source.replace("assert dest.exists() and sha(dest)==reuse['sha256']","assert not dest.exists();shutil.copy2(native,dest);assert sha(dest)==reuse['sha256']")
source=source.replace('profile_v2','smoke').replace('profile-v2','smoke').replace('profile_results_v2','results')
source=source.replace("*(A/'states_v2').glob('*')","A/'manifest.json'")
(A/'execute_smoke.py').write_text(source,encoding='utf-8');put(A/'execute_smoke.py','execute_smoke.py')
manifest=dict(files=files,provenance=provenance,native_reuse=reuse,input_sources={str(old/'inputs/selected.jsonl'):sha(old/'inputs/selected.jsonl')},
    raw_checks=checks,planned_calls=8,physical_measurement=False,contract_conversion='original r444 estimated_record/estimated_header unchanged')
(B/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
shutil.copy2(B/'manifest.json',A/'manifest.json')
with zipfile.ZipFile(A/'bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in B.rglob('*'):
        if p.is_file():z.write(p,p.relative_to(B).as_posix())
print(json.dumps(dict(files=len(files),archive_sha256=sha(A/'bundle.zip'),raw_checks=checks)))
