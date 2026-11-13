"""소수 저장 입력과 최신 정책 사본을 준비한다. Evaluator/native/ROS 계산은 하지 않는다."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

from contract_tools import load_converters, assert_snapshot_numbers_unchanged

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
DAY=A.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2)
        stream.write('\n')


def main():
    bundle=A/'bundle'
    bundle.mkdir(exist_ok=False)
    files,provenance={},{}
    def put(source,name):
        dest=bundle/name
        dest.parent.mkdir(parents=True,exist_ok=True)
        before=sha(source)
        shutil.copyfile(source,dest)
        assert sha(dest)==before==sha(source)
        files[name]=before
        provenance[name]=dict(source=source.relative_to(ROOT).as_posix(),sha256=before)
    def generated(name,value):
        write(bundle/name,value)
        files[name]=sha(bundle/name)
    for pkg in ('arena_autonomy','arena_vehicle_interface'):
        for source in sorted((ROOT/'src'/pkg/pkg).rglob('*')):
            if source.is_file() and source.suffix in ('.py','.cpp','.hpp','.h'):
                put(source,source.relative_to(ROOT).as_posix())
    put(ROOT/'src/arena_autonomy/setup.py','src/arena_autonomy/setup.py')
    for name in ('contract_tools.py','run_contract.py','execute_package.py'):
        put(A/name,name)
    put(DAY/'estimated_observation_r444_v1/verify_saved.py','contract_reference.py')
    put(DAY/'runtime_cost_r411_v1/diff_utils.py','diff_utils.py')
    put(DAY/'estimated_observation_cost_r447_v1/remote_bootstrap.py','environment_reference.py')
    selection={
        'r457':[(1360,'new_bounded_guard_current'),(1986,'new_right_mask_stop')],
        'r452':[(519,'existing_guard_positive'),(539,'negative_measured_positive'),(531,'finite_new_side_stop')],
    }
    records=[]
    sources={}
    checkpoint=None
    for dataset,choices in selection.items():
        folder='solo_full_lap_r457_v1' if dataset=='r457' else 'guard_crawl_drive_r452_v1'
        path=DAY/folder/'run1/follow_decisions.jsonl'
        sources[path.relative_to(ROOT).as_posix()]=sha(path)
        wanted=dict(choices)
        extracted={}
        header=None
        for line in path.open(encoding='utf8'):
            event=json.loads(line)
            if event['kind']=='runtime_evaluator_header':
                assert header is None
                header=event['data']['header']
            if dataset=='r452' and event['kind']=='runtime_plan_submitted':
                d=event['data']
                if (d['generation'],d['sequence'])==(360,573):
                    checkpoint=d['input']
            if event['kind']=='call' and event['seq'] in wanted:
                d=event['data']
                current=d['current_rebase']
                compute=d['compute']
                extracted[event['seq']]=dict(id=f'{dataset}:{event["seq"]}',dataset=dataset,trace_seq=event['seq'],
                    role=wanted[event['seq']],plan_input=compute['input'],plan_output=d['output'],
                    current=current['input'],retained=compute['route_check']['retained'],expected=current['output'],
                    identity=dict(generation=compute['generation'],sequence=compute['sequence'],
                        plan_reference_s=compute['input']['reference_s'],current_reference_s=current['input']['reference_s']),
                    actual_recorded_current=True,current_synthesized=False)
        assert set(extracted)==set(wanted) and header is not None
        records.extend(extracted[seq] for seq,_ in choices)
        generated(f'inputs/{dataset}_header.json',header)
        assert sources[path.relative_to(ROOT).as_posix()]==sha(path)
    assert records[0]['expected']['speed_mps']==.15
    assert records[0]['expected']['current_approach']['basis']=='observed_with_mask_budget'
    assert records[3]['current']['speed_mps']<0 and records[3]['expected']['speed_mps']>0
    assert all(records[i]['expected']['speed_mps']==0 for i in (1,4))
    assert checkpoint['reference_s']==35.22 and checkpoint['speed_mps']==0
    generated('inputs/plan_checkpoint_input.json',checkpoint)
    checkpoint_file=DAY/'guard_crawl_entry_r456_v1/pinpoint_v1/local_actual_snapshots.pkl'
    assert sha(checkpoint_file)==read(DAY/'bounded_guard_approach_r459_v1/local_windows_v1/summary.json')['source_sha256']
    put(checkpoint_file,'inputs/actual_owner_snapshots.pkl')
    output=bundle/'inputs/currents.jsonl'
    output.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in records),encoding='utf8')
    files['inputs/currents.jsonl']=sha(output)
    converter=load_converters(bundle/'contract_reference.py')
    for record in records:
        original=copy.deepcopy(record)
        converted=converter['estimated_record'](record)
        for name in ('plan_input','current'):
            assert_snapshot_numbers_unchanged(record[name],converted[name])
        assert record==original
    assert_snapshot_numbers_unchanged(checkpoint,converter['estimated_snapshot'](checkpoint))
    integrations=read(DAY/'bounded_guard_approach_r459_v1/integration.json')
    assert integrations['integrated'] is True
    for row in integrations['files']:
        assert sha(ROOT/row['path'])==row['after'],row['path']
    old=read(DAY/'estimated_observation_cost_r447_v1/source_manifest.json')['files']
    native_names=[p for p in old if '/_native/' in p or p.endswith('/setup.py')]
    assert len(native_names)==4 and all(sha(ROOT/p)==old[p] for p in native_names)
    for path in bundle.rglob('*.py'):
        ast.parse(path.read_bytes())
    selected=[dict(id=r['id'],role=r['role'],identity=r['identity'],measured_mps=r['current']['speed_mps'],
        expected_mps=r['expected']['speed_mps'],expected_reason=r['expected']['reason']) for r in records]
    manifest=dict(files=files,provenance=provenance,raw_source_hashes=sources,selected=selected,
        policy_integration=integrations['files'],policy_changes=[],
        planned_current_calls=10,planned_local_checkpoint_calls=2,planned_invalid_contract_calls=1,
        full_stateful_plan_replay=False,checkpoint_identity=dict(generation=360,sequence=573,reference_s=35.22),
        checkpoint_scope='actual owner snapshot local guard branch only; not whole plan or current',
        numeric_inputs_unchanged=True,contract_only_snapshot_checks=11,raw_snapshot_count=11,
        physical_measurement=False,current_synthesized=False,execution_pending_resource_assignment=True,
        native_reuse=dict(source='/home/q/it-arena-r447-a/src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so',
            sha256='a2afb70525be1e86f63ae072abc4fbaeec10d65f0596e5f642f135ba6a1de394',
            original_source_sha256={p:old[p] for p in native_names},new_build_planned=False))
    write(bundle/'source_manifest.json',manifest)
    write(A/'source_manifest.json',manifest)
    archive=A/'bundle_sources.zip'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():z.write(path,path.relative_to(bundle).as_posix())
    receipt=dict(files=len(files),archive_sha256=sha(archive),manifest_sha256=sha(A/'source_manifest.json'),
        selected=selected,planned_total_computations=13,current_cases=5,local_plan_checkpoint_cases=1,
        source_policy_changes=0,calculation_executed=False,remote_transfer_done=False,resource_assignment_pending=True)
    write(A/'prepared.json',receipt)
    print(json.dumps(receipt,ensure_ascii=False))


if __name__=='__main__':
    main()
