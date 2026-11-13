"""r0 불변 번들에서 송신 한계 진단 r1만 준비. 제어 계산/빌드/회귀 실행 없음."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;P=A.parent
R0='223c2df288f25f2147b78ef26a47bc235ee7929cd3be3b346ec43fae0bab265e'


def digest(raw):return hashlib.sha256(raw).hexdigest()


def transplant(base,stage,frozen,name):
    before=base.decode('utf8').replace('\r\n','\n').splitlines(True)
    after=stage.decode('utf8').replace('\r\n','\n').splitlines(True)
    result=frozen.decode('utf8').replace('\r\n','\n')
    for group in difflib.SequenceMatcher(a=before,b=after,autojunk=False).get_grouped_opcodes(3):
        old=''.join(before[group[0][1]:group[-1][2]]);new=''.join(after[group[0][3]:group[-1][4]])
        assert old and result.count(old)==1,('frozen hunk missing/ambiguous',name)
        result=result.replace(old,new,1)
    return result.encode('utf8')


def main():
    destination=A/'bundle_sources.zip';manifest_path=A/'source_manifest.json'
    assert not destination.exists() and not manifest_path.exists(),'preserve r1 bundle'
    old=P/'bundle_sources.zip';assert digest(old.read_bytes())==R0
    with zipfile.ZipFile(old) as archive:
        prior=json.loads(archive.read('r388/source_manifest.json'))
        assert all(digest(archive.read(n))==h for n,h in prior['files'].items())
        payload={name:archive.read(name) for name in archive.namelist() if name.startswith(('frozen/','geometry/'))}
    stage_manifest={};patch=[];transplanted={}
    for path in sorted((A/'staged').rglob('*.py')):
        name=path.relative_to(A/'staged').as_posix();base=A/'base'/name
        new=path.read_bytes();before=base.read_bytes() if base.exists() else b''
        ast.parse(new,filename=name)
        if before:
            assert before==(P/'base'/name).read_bytes()
            changed=transplant(before,new,payload['frozen/'+name],name)
        else:
            assert 'frozen/'+name not in payload;changed=new
        payload['geometry/'+name]=changed
        stage_manifest[name]=dict(base_sha256=digest(before) if before else None,staged_sha256=digest(new))
        transplanted[name]=dict(frozen_sha256=digest(payload['frozen/'+name]) if before else None,geometry_sha256=digest(changed))
        patch.extend(difflib.unified_diff(before.decode('utf8').replace('\r\n','\n').splitlines(True),
            new.decode('utf8').replace('\r\n','\n').splitlines(True),fromfile='a/'+name if before else '/dev/null',tofile='b/'+name))
    assert len(stage_manifest)==9
    (A/'stage_manifest.json').write_text(json.dumps(dict(files=stage_manifest,common_sources_modified=False),indent=2)+'\n',encoding='utf8')
    (A/'geometry_contract.patch').write_text(''.join(patch),encoding='utf8',newline='\n')
    # 원 launch 함수 본문을 축약/수치 변경 없이 하네스에 보존했는지 AST로 확인한다.
    launch=payload['frozen/src/arena_bringup/launch/simulation.launch.py']
    original=next(n for n in ast.parse(launch).body if isinstance(n,ast.FunctionDef) and n.name=='_float32_limit_not_above')
    preserved=next(n for n in ast.parse((A/'original_float32_limit.py').read_bytes()).body if isinstance(n,ast.FunctionDef))
    assert ast.dump(original)==ast.dump(preserved)
    shared=next(n for n in ast.parse((A/'staged/src/arena_autonomy/arena_autonomy/vehicle_geometry.py').read_bytes()).body
        if isinstance(n,ast.FunctionDef) and n.name=='float32_limit_not_above')
    shared.name=original.name;assert ast.dump(original)==ast.dump(shared)
    for path in A.iterdir():
        if path.is_file() and path.suffix in ('.py','.jsonl','.json','.md','.patch'):
            payload['r388/'+path.name]=path.read_bytes()
    for sub in ('base','staged'):
        for path in (A/sub).rglob('*.py'):payload['r388/'+sub+'/'+path.relative_to(A/sub).as_posix()]=path.read_bytes()
    for name,raw in payload.items():
        if name.endswith('.py'):ast.parse(raw,filename=name)
    inventory={n:digest(raw) for n,raw in sorted(payload.items())}
    frozen={n.removeprefix('frozen/'):v for n,v in payload.items() if n.startswith('frozen/')}
    geometry={n.removeprefix('geometry/'):v for n,v in payload.items() if n.startswith('geometry/') and not n.startswith('geometry/tests/')}
    assert {n for n in geometry if n not in frozen or frozen[n]!=geometry[n]}==set(stage_manifest)
    manifest=dict(schema='r388_r1_float32_sender_contract',files=inventory,original_r0_archive_sha256=R0,
        original_r0_result_zip_sha256=digest((P/'remote_result/results.zip').read_bytes()),
        original_r0_pipeline_passed=False,original_r0_sources_preserved=True,
        original_r377_archive_sha256=prior['source_r377_archive_sha256'],frozen_source_count=len(frozen),
        geometry_only_transplants=transplanted,candidate_chain=prior['candidate_chain'],saved_input=prior['saved_input'],
        preserved_original_float32_ast_equal=True,shared_float32_body_ast_equal=True,
        stored_steering_and_wheelbase_changed=False,r386_candidate_included=False)
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(destination,'x',zipfile.ZIP_DEFLATED,compresslevel=1) as archive:
        for name,raw in sorted(payload.items()):archive.writestr(name,raw)
        archive.writestr('r388/source_manifest.json',manifest_path.read_bytes())
    with zipfile.ZipFile(destination) as archive:
        assert all(digest(archive.read(n))==h for n,h in inventory.items())
    receipt=dict(prepared_only=True,files=len(inventory),archive_bytes=destination.stat().st_size,
        archive_sha256=digest(destination.read_bytes()),manifest_sha256=digest(manifest_path.read_bytes()),
        bootstrap_sha256=digest((A/'execute_isolated.py').read_bytes()),patch_sha256=digest((A/'geometry_contract.patch').read_bytes()),
        original_r0_archive_sha256=R0,controller_calls=0,tests_executed=False,native_built=False,devices_opened=False)
    (A/'bundle_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8');print(json.dumps(receipt))


if __name__=='__main__':main()
