"""원 r418 동결본과313 current 입력을 보존하는 정적 준비. 제어기 import/실행 없음."""
import ast
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
SOURCE=HERE.parent/'crop_edge_drive_r418_v1'
TRACE_SHA='67bd68142f4d0596ed60bd89fe280fd5b9a3e60e79802d3d7d0d1d6da6ba30da'


def sha(value):return hashlib.sha256(value).hexdigest()
def encoded(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def pretty(value):return (json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8')


def main():
    assert not (HERE/'bundle_sources.zip').exists()
    trace=SOURCE/'run1/follow_decisions.jsonl';assert sha(trace.read_bytes())==TRACE_SHA
    audit=HERE.parent/'crop_edge_stall_r421_v1/verification_final.json'
    assert sha(audit.read_bytes())=='23f14cc02ae5cb40c91e2d4f4e8e72415987dba7cfdfd5062b8f651c7613e6ad'
    records=[];seen=set();header=factory=None
    for line in trace.open(encoding='utf8'):
        event=json.loads(line);data=event['data']
        if event['kind']=='header':header=data
        elif event['kind']=='runtime_factory_policy':factory=data
        elif event['kind']=='call' and data.get('current_rebase') is not None:
            rebase=data['current_rebase'];compute=rebase['compute'];plan=compute['plan']
            key=(compute['generation'],compute['sequence']);assert key not in seen;seen.add(key)
            assert plan['input']==data['input'] and plan['output']==data['output']
            assert rebase['input']==compute['input'] and compute['route_check']==plan['route_check']
            record=dict(index=len(records),trace_seq=event['seq'],generation=key[0],sequence=key[1],
                plan_generation=plan['generation'],plan_sequence=plan['sequence'],plan_input=plan['input'],
                plan_output=plan['output'],current=rebase['input'],retained=compute['route_check']['retained'],
                expected=rebase['output'])
            record['input_sha256']=sha(encoded({k:record[k] for k in ('plan_input','plan_output','current','retained')}))
            records.append(record)
    assert len(records)==313 and header['follow_config']['crop_edge_component_trial'] is True
    assert factory['policy']=='cad_runtime_r377_v1' and factory['motion_time_policy']=='verified_only'
    assert factory['motion_timing_verified'] is True and factory['current_corner_budget']=='r396_v1'
    representatives=[r for r in records if r['trace_seq']==1003];assert len(representatives)==1
    selected=dict(header,runtime_policy=factory['policy'],runtime_time_policy=factory['motion_time_policy'],
        numeric_execution='native_v1',current_corner_budget='r396_v1',current_corner_environment=dict(
            use_sim_time=True,motion_timing_verified=True,backend='native2slot_v1'))
    # 원 source archive를 읽어 common의 후속 변경을 옛 결과의 baseline에 섞지 않는다.
    with zipfile.ZipFile(SOURCE/'sources.zip') as z:original={n:z.read(n) for n in z.namelist()}
    staged={p.relative_to(HERE/'staged').as_posix():p.read_bytes() for p in (HERE/'staged').rglob('*.py')}
    assert len(staged)==8
    overlays={**original,**staged,'tests/test_crop_current_scope.py':(HERE/'tests/test_crop_current_scope.py').read_bytes()}
    spec=importlib.util.spec_from_file_location('r423_closure',ROOT/'scripts/python_dependency_closure.py')
    closure=importlib.util.module_from_spec(spec);spec.loader.exec_module(closure)
    entries=list(staged)+['tests/test_crop_current_scope.py']
    result=closure.collect(entries,roots=closure.PACKAGE_ROOTS+closure.TEST_ROOTS,members=overlays)
    source_names=[n for n in result['files'] if n.startswith('src/')]
    members={};ledger={};patch=[];common_mismatches=[]
    for name in source_names:
        if name in original:
            members['baseline/'+name]=original[name]
            if not (ROOT/name).is_file() or (ROOT/name).read_bytes()!=original[name]:common_mismatches.append(name)
        members['candidate/'+name]=overlays[name]
    assert not common_mismatches,('common differs from r418 frozen import closure',common_mismatches)
    for name,after in sorted(staged.items()):
        before=original.get(name,b'')
        ledger[name]=dict(base_sha256=sha(before) if before else None,staged_sha256=sha(after),new_file=not before)
        patch.extend(difflib.unified_diff(before.decode('utf8').replace('\r\n','\n').splitlines(True),
            after.decode('utf8').replace('\r\n','\n').splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    provenance=dict(trace_path=trace.relative_to(ROOT).as_posix(),trace_sha256=TRACE_SHA,
        r421_verification_sha256=sha(audit.read_bytes()),r418_sources_sha256=sha((SOURCE/'sources.zip').read_bytes()),
        selected_current=313,unique_current=313,representative_trace_seq=1003,
        representative_index=representatives[0]['index'],same_raw_scan=sum(r['plan_input']['scan']==r['current']['scan'] for r in records),
        original_current_positive=sum(r['expected']['speed_mps']>0 for r in records),
        extraction='each current_rebase with checked nested plan input/output identity; no omitted current',
        source='r418 frozen import closure; identical to current common before proposed overlay',
        header_derivation='raw header + recorded runtime factory selection + validated r418 native simulation launch contract',
        added_header_fields={k:selected[k] for k in ('runtime_policy','runtime_time_policy','numeric_execution','current_corner_environment')},
        representative_first_then_remaining312=True,original_plan_history_replayed=False,
        real_sensors=False,permission_reused=False)
    inputs={'header.json':pretty(selected),'raw_header.json':pretty(header),'factory.json':pretty(factory),
        'representative.json':pretty(representatives[0]),'currents.jsonl':b''.join(encoded(r)+b'\n' for r in records),
        'provenance.json':pretty(provenance)}
    (HERE/'inputs').mkdir()
    for name,data in inputs.items():(HERE/'inputs'/name).write_bytes(data);members['inputs/'+name]=data
    for name in ('support.py','run_arm.py','audited_evaluator.py'):members[name]=(HERE/name).read_bytes()
    members['tests/test_crop_current_scope.py']=(HERE/'tests/test_crop_current_scope.py').read_bytes()
    for name,data in members.items():
        if name.endswith('.py'):ast.parse(data,filename=name)
    for p in (HERE/'execute_isolated.py',HERE/'prepare.py'):ast.parse(p.read_bytes(),filename=str(p))
    manifest=dict(schema='crop_current_scope_r423_bundle_v1',files={n:sha(d) for n,d in sorted(members.items())},
        source_ledger=ledger,provenance=provenance,static_ast_passed=True,control_calls=0)
    (HERE/'source_ledger.json').write_bytes(pretty(ledger));(HERE/'source_changes.patch').write_text(''.join(patch),encoding='utf8',newline='\n')
    (HERE/'bundle_manifest.json').write_bytes(pretty(manifest))
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,data in sorted(members.items()):z.writestr(name,data)
        z.writestr('bundle_manifest.json',pretty(manifest))
    assert sha(trace.read_bytes())==TRACE_SHA
    preparation_files=[p for p in HERE.rglob('*') if p.is_file() and p.name!='preparation_manifest.json']
    preparation=dict(files={p.relative_to(HERE).as_posix():sha(p.read_bytes()) for p in sorted(preparation_files)},
        source_count=len(source_names),staged_count=len(staged),control_calls=0,windows_static_only=True)
    (HERE/'preparation_manifest.json').write_bytes(pretty(preparation))
    print(json.dumps(dict(staged=8,source_modules=len(source_names),bundle_files=len(members),currents=313,
        representative_index=representatives[0]['index'],bundle_sha256=sha((HERE/'bundle_sources.zip').read_bytes()),
        manifest_sha256=sha((HERE/'bundle_manifest.json').read_bytes()),control_calls=0)))


if __name__=='__main__':main()
