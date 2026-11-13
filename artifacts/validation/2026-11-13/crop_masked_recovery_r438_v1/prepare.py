"""r437 동결 소스와 저장된 current 입력만 모은다. 공통/원 입력 수정 없음."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
R437=ROOT.parent/'applied_limit_drive_r437_v1';R426=ROOT.parent/'crop_current_reach_r426_v1'
def sha(data):return hashlib.sha256(data).hexdigest()
def encoded(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def write(path,value):
    with path.open('x',encoding='utf-8') as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)

inputs=ROOT/'inputs';inputs.mkdir(exist_ok=False)
records=[];header=factory=None
trace=R437/'run1/follow_decisions.jsonl'
for line in trace.open(encoding='utf-8'):
    event=json.loads(line);data=event['data']
    if event['kind']=='header':header=data
    elif event['kind']=='runtime_factory_policy':factory=data
    elif event['kind']=='call' and data.get('current_rebase') is not None:
        reb=data['current_rebase'];expected=reb['output']
        if not expected.get('current_crop_component_retry'):continue
        compute=reb['compute'];plan=compute['plan']
        assert plan['input']==data['input'] and plan['output']==data['output'] and reb['input']==compute['input']
        records.append(dict(index=len(records),trace_seq=event['seq'],generation=compute['generation'],
            sequence=compute['sequence'],plan_input=plan['input'],plan_output=plan['output'],
            current=reb['input'],retained=compute['route_check']['retained'],expected=expected))
assert len(records)==96 and factory['policy']=='cad_runtime_r377_v1' and factory['motion_time_policy']=='verified_only'
selected=dict(header,runtime_policy=factory['policy'],runtime_time_policy=factory['motion_time_policy'],
    numeric_execution='native_v1',current_corner_environment=dict(use_sim_time=True,motion_timing_verified=True,backend='native2slot_v1'))
write(inputs/'r437_header.json',selected)
expected426={}
with zipfile.ZipFile(R426/'verify_v1/conditional-full.zip') as z:
    for name in ('results/remaining_candidate/rows.jsonl','results/representative_candidate/rows.jsonl'):
        for line in z.read(name).decode().splitlines():
            row=json.loads(line);expected426[row['trace_seq']]=row['output']
assert len(expected426)==313
old=[json.loads(line) for line in (R426/'inputs/currents.jsonl').open(encoding='utf-8')]
assert len(old)==313
for row in old:row['expected']=expected426[row['trace_seq']]
write(inputs/'r426_header.json',json.loads((R426/'inputs/header.json').read_bytes()))
for dataset,rows in (('r437',records),('r426',old)):
    with (inputs/(dataset+'_currents.jsonl')).open('xb') as stream:
        for row in rows:
            row['dataset']=dataset
            row['input_sha256']=sha(encoded({key:row[key] for key in ('plan_input','plan_output','current','retained')}))
            stream.write(encoded(row)+b'\n')
with zipfile.ZipFile(R437/'sources.zip') as z:
    files={name:z.read(name) for name in z.namelist() if name.startswith((
        'src/arena_autonomy/arena_autonomy/','src/arena_vehicle_interface/arena_vehicle_interface/')) and name.endswith('.py')}
    crop=z.read('src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_candidate.py').decode('utf-8')
    numeric={name:sha(z.read(name)) for name in z.namelist() if 'numeric' in name and name.startswith('src/arena_autonomy/')}
candidate=crop.replace('registered_environment,core_rebase):','registered_environment,core_rebase,trial_cap_mps=.2):')
assert candidate!=crop and candidate.count('target=min(output[0],.2)')==1 and candidate.count('trial_target_cap_mps=.2,')==1
candidate=candidate.replace('target=min(output[0],.2)','target=min(output[0],trial_cap_mps)').replace('trial_target_cap_mps=.2,','trial_target_cap_mps=trial_cap_mps,')
candidate=candidate.replace('from .crop_current_scope import','from arena_autonomy.race_runtime_policy.crop_current_scope import').replace('from .crop_current_proof import','from arena_autonomy.race_runtime_policy.crop_current_proof import')
(ROOT/'stage').mkdir(exist_ok=True)
(ROOT/'stage/fixed_target_candidate.py').write_text(candidate,encoding='utf-8')
binary=next((REPO/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'))
binary_name='src/arena_autonomy/arena_autonomy/'+binary.name
files[binary_name]=binary.read_bytes()
with zipfile.ZipFile(ROOT/'frozen_packages.zip','x',zipfile.ZIP_DEFLATED) as z:
    for name,data in files.items():z.writestr(name,data)
provenance=dict(source_archive_sha256=sha((R437/'sources.zip').read_bytes()),trace_sha256=sha(trace.read_bytes()),
    r426_currents_sha256=sha((R426/'inputs/currents.jsonl').read_bytes()),
    r426_expected_archive_sha256=sha((R426/'verify_v1/conditional-full.zip').read_bytes()),
    frozen_package_files={name:sha(data) for name,data in files.items()},numeric_source_sha256=numeric,
    copied_native_binary=dict(path=str(binary.relative_to(REPO)),sha256=sha(binary.read_bytes())),
    inputs={path.name:sha(path.read_bytes()) for path in inputs.iterdir()},
    original_crop_sha256=sha(crop.encode()),stage_sha256=sha(candidate.encode()),
    r437_cases=len(records),r426_cases=len(old),no_common_changes=True)
write(ROOT/'preparation.json',provenance)
print(json.dumps({k:provenance[k] for k in ('r437_cases','r426_cases','copied_native_binary','numeric_source_sha256')}))
