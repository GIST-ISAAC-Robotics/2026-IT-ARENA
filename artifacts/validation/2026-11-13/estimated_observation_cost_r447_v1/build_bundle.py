"""Windows에서 최신 공통 사본/원 입력/계약 변환을 동결. 계산/장치 실행은 하지 않는다."""
from pathlib import Path
import ast,copy,hashlib,json,shutil,zipfile
BASE=Path(__file__).resolve().parent;REPO=BASE.parents[3];DAY=BASE.parent
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,value):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,indent=2)+'\n',encoding='utf8')
def main():
    bundle=BASE/'bundle';bundle.mkdir(exist_ok=False);files={};provenance={}
    def put(source,name):
        dest=bundle/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,dest)
        files[name]=digest(dest);provenance[name]=dict(source=source.relative_to(REPO).as_posix(),sha256=digest(source))
    for pkg in ('arena_autonomy','arena_vehicle_interface'):
        for p in sorted((REPO/'src'/pkg/pkg).rglob('*')):
            if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h'):put(p,p.relative_to(REPO).as_posix())
    for name in ('setup.py','package.xml','resource/arena_autonomy'):
        p=REPO/'src/arena_autonomy'/name;put(p,p.relative_to(REPO).as_posix())
    for name in ('run_cost.py','remote_bootstrap.py','import_preflight.py'):put(BASE/name,name)
    put(DAY/'estimated_observation_r444_v1/verify_saved.py','contract_reference.py')
    put(DAY/'runtime_cost_r411_v1/diff_utils.py','diff_utils.py')
    put(BASE/'verify_nodes.py','node_check/verify_nodes.py')
    for n in ('runtime_profile.yaml','bindings.json'):
        put(DAY/'estimated_observation_r444_v1/verify_nodes_v1'/n,'node_check/'+n)
    data=DAY/'crop_masked_recovery_r438_v1/inputs'
    records=[]
    for d in ('r437','r426'):
        put(data/(d+'_header.json'),'inputs/'+d+'_header.json')
        rows=list(map(json.loads,(data/(d+'_currents.jsonl')).open()))
        if d=='r437':
            assert len(rows)==96
            for r in rows:r['selected_role']='r437_all_current'
            records+=rows
        else:
            for seq,role in ((369,'existing_normal_positive'),(680,'existing_corner_target'),(614,'negative_measured')):
                r=next(r for r in rows if r['trace_seq']==seq);r['selected_role']=role;records.append(r)
    assert len(records)==99 and len({(r['dataset'],r['trace_seq']) for r in records})==99
    selected=bundle/'inputs/selected.jsonl'
    selected.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in records),encoding='utf8')
    files['inputs/selected.jsonl']=digest(selected)
    all_expected={r['dataset']+':'+str(r['trace_seq']):r['output'] for r in map(json.loads,
        (DAY/'crop_current_crawl_r440_v1/verify_v1/saved_result/rows.jsonl').open())}
    expected={r['dataset']+':'+str(r['trace_seq']):all_expected[r['dataset']+':'+str(r['trace_seq'])] for r in records}
    assert expected['r426:369']['speed_mps']>.1 and expected['r426:680']['current_corner_distance_budget']['applied'] is True
    assert records[-1]['current']['speed_mps']<0
    write(bundle/'inputs/x86_expected.json',expected);files['inputs/x86_expected.json']=digest(bundle/'inputs/x86_expected.json')
    # 변환 함수 원문만 실행: 원시 수치/측정 부호/scan/epoch 보존 여부를 Windows에서 대조한다.
    tree=ast.parse((bundle/'contract_reference.py').read_text(encoding='utf8'))
    funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='estimated_record']
    namespace=dict(copy=copy,ESTIMATED_SCOPE='host_estimated_crop_v1',component_scope=lambda s:'test_scope:'+s)
    exec(compile(ast.Module(body=funcs,type_ignores=[]),'contract_reference.py','exec'),namespace)
    for original in records:
        before=copy.deepcopy(original);converted=namespace['estimated_record'](original)
        assert original==before
        for snapshot in ('plan_input','current'):
            assert converted[snapshot]['timing_verified'] is False
            assert converted[snapshot]['timing_policy']=='host_estimated_v1'
            assert converted[snapshot]['scan']['measurements']['time_basis']=='unverified_estimate'
            for key in ('timing_verified','timing_policy'):
                if key in original[snapshot]:converted[snapshot][key]=original[snapshot][key]
                else:converted[snapshot].pop(key,None)
            converted[snapshot]['scan']['measurements']['time_basis']=original[snapshot]['scan']['measurements']['time_basis']
        component=converted['plan_output'][2].get('normal',{}).get('partial_observation',{}).get('crop_edge_component')
        if component is not None:component['scope']=original['plan_output'][2]['normal']['partial_observation']['crop_edge_component']['scope']
        assert converted==original,'numeric/raw input changed'
    # r444 기본 이관과 r446 입구 계약의 최신 SHA를 대조한다.
    refs=json.loads((DAY/'estimated_observation_r444_v1/integration.json').read_bytes())['files']
    refs.update(json.loads((DAY/'estimated_entry_contract_r446_v1/changes.json').read_bytes())['files'])
    for name,ref in refs.items():assert digest(REPO/name)==ref['stage_sha256'],name
    parsed=0
    for p in bundle.rglob('*.py'):ast.parse(p.read_text(encoding='utf8'));parsed+=1
    manifest=dict(files=files,provenance=provenance,selection_count=99,contract_count=2,planned_evaluator_calls=198,
        additional_warmup_calls=0,source_contract='stored simulation numerical inputs',physical_measurement=False,
        common_integrations_verified=list(refs),python_syntax_files=parsed,contract_copies_checked=99)
    write(bundle/'source_manifest.json',manifest);shutil.copyfile(bundle/'source_manifest.json',BASE/'source_manifest.json')
    archive=BASE/'bundle_sources.zip'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(bundle.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(bundle).as_posix())
    receipt=dict(archive_sha256=digest(archive),manifest_sha256=digest(BASE/'source_manifest.json'),
        files=len(files),python_syntax_files=parsed,contract_copies_checked=99,selected=99,
        representatives=[dict(dataset=r['dataset'],trace_seq=r['trace_seq'],role=r['selected_role'],
            measured_mps=r['current']['speed_mps'],x86_output_mps=expected[r['dataset']+':'+str(r['trace_seq'])]['speed_mps']) for r in records[-3:]])
    write(BASE/'prepared.json',receipt);print(json.dumps(receipt))
if __name__=='__main__':main()
