"""r377 원문 두 arm과 기하 변경만 묶는다. 제어 계산/시험/빌드 실행 없음."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
CHAIN=(
    'artifacts/validation/2026-11-13/released_current_r375_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_corner_r370_v1/candidate.py',
    'artifacts/validation/2026-11-13/solo_crawl_retry_r352_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_gap_r346_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_low_r343_v1/candidate.py',
    'artifacts/validation/2026-11-12/current_region_policy_r281_v1/candidate.py',
    'artifacts/validation/2026-11-12/retained_route_execution_r266_v1/candidate_policy.py')


def digest(raw):return hashlib.sha256(raw).hexdigest()


def transplant(base,stage,frozen,name):
    """현재 전체 파일을 복사하지 않고, 보존 원문에 정확히 맞는 변경 hunk만 이관한다."""
    before=base.decode('utf8').replace('\r\n','\n').splitlines(True)
    after=stage.decode('utf8').replace('\r\n','\n').splitlines(True)
    result=frozen.decode('utf8').replace('\r\n','\n')
    for group in difflib.SequenceMatcher(a=before,b=after,autojunk=False).get_grouped_opcodes(3):
        old=''.join(before[group[0][1]:group[-1][2]]);new=''.join(after[group[0][3]:group[-1][4]])
        assert old and result.count(old)==1,('frozen hunk missing/ambiguous',name,old[:100])
        result=result.replace(old,new,1)
    ast.parse(result,filename=name)
    return result.encode('utf8')


def main():
    destination=A/'bundle_sources.zip';manifest_path=A/'source_manifest.json'
    assert not destination.exists() and not manifest_path.exists(),'preserve prepared archive'
    stage=json.loads((A/'stage_manifest.json').read_bytes())['files']
    old=A.parent/'released_cad_drive_r377_v1';registered=json.loads((old/'registered.json').read_bytes())
    assert digest((old/'sources.zip').read_bytes())==registered['source_archive_sha256']
    extras=set(stage)-{'src/arena_autonomy/arena_autonomy/vehicle_geometry.py'}
    extras.update(('src/arena_description/config/vehicle.final_cad_2026_11_13.yaml',
        'src/arena_description/config/vehicle.yaml',
        'artifacts/validation/2026-11-13/released_current_r375_v1/compare.py'))
    frozen={}
    with zipfile.ZipFile(old/'sources.zip') as archive:
        for name in archive.namelist():
            wanted=(name.startswith(('src/arena_autonomy/','src/arena_vehicle_interface/',
                'artifacts/validation/2026-11-12/retained_route_execution_r266_v1/'))
                and Path(name).suffix in ('.py','.cpp','.hpp','.h','.xml','.cfg')) or name in CHAIN or name in extras
            if wanted:
                raw=archive.read(name);assert digest(raw)==registered['input_sha256'][name]
                frozen[name]=raw
    assert (extras|set(CHAIN))<=frozen.keys()
    geometry=dict(frozen);transplanted={}
    for name,entry in stage.items():
        new=(A/'staged'/name).read_bytes();assert digest(new)==entry['staged_sha256']
        if entry['base_sha256'] is None:
            assert name not in frozen;geometry[name]=new
        else:
            base=(A/'base'/name).read_bytes();assert digest(base)==entry['base_sha256']
            assert (ROOT/name).read_bytes()==base,('common source changed while preparing',name)
            geometry[name]=transplant(base,new,frozen[name],name)
        transplanted[name]=dict(frozen_sha256=digest(frozen[name]) if name in frozen else None,
            geometry_sha256=digest(geometry[name]),stage_sha256=entry['staged_sha256'])
    assert {n for n in geometry if n not in frozen or frozen[n]!=geometry[n]}==set(stage)
    trace=old/'run1/follow_decisions.jsonl';audit=json.loads((old/'audit.json').read_bytes())
    assert digest(trace.read_bytes())==audit['input_sha256']['run1/follow_decisions.jsonl']
    header=None;last=None;count=0;header_lineno=last_lineno=None
    for lineno,line in enumerate(trace.open('rb'),1):
        event=json.loads(line)
        if event['kind']=='header':header=line;header_lineno=lineno
        if event['kind']=='call' and 'current_rebase' in event['data']:
            last=line;last_lineno=lineno;count+=1
    assert count==223 and header and last
    (A/'input_header.jsonl').write_bytes(header);(A/'input_last_call.jsonl').write_bytes(last)
    input_manifest=dict(original_trace_sha256=audit['input_sha256']['run1/follow_decisions.jsonl'],
        selected_index=222,current_count=count,header_line=header_lineno,last_line=last_lineno,
        header_sha256=digest(header),last_call_sha256=digest(last),original_input_rewritten=False)
    (A/'input_manifest.json').write_text(json.dumps(input_manifest,indent=2)+'\n',encoding='utf8')
    payload={**{'frozen/'+n:v for n,v in frozen.items()},**{'geometry/'+n:v for n,v in geometry.items()}}
    # 순수 하드웨어 명령 회귀는 원소스에 있는 원 테스트를 함께 실행한다.
    for name in ('tests/test_hardware_preview_runner.py',):
        payload['geometry/'+name]=(ROOT/name).read_bytes()
    for path in A.iterdir():
        if path.is_file() and path.suffix in ('.py','.md','.jsonl','.patch','.json'):
            payload['r388/'+path.name]=path.read_bytes()
    for path in (A/'base').rglob('*.py'):payload['r388/base/'+path.relative_to(A/'base').as_posix()]=path.read_bytes()
    for path in (A/'staged').rglob('*.py'):payload['r388/staged/'+path.relative_to(A/'staged').as_posix()]=path.read_bytes()
    for name,raw in payload.items():
        if name.endswith('.py'):ast.parse(raw,filename=name)
    inventory={n:digest(raw) for n,raw in sorted(payload.items())}
    manifest=dict(schema='r388_frozen_r377_geometry_only_bundle',files=inventory,
        source_r377_archive_sha256=registered['source_archive_sha256'],frozen_source_count=len(frozen),
        geometry_only_transplants=transplanted,candidate_chain=list(CHAIN),saved_input=input_manifest,
        current_common_control_sources_imported=False,r379_change_included=False,r386_candidate_included=False)
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(destination,'x',zipfile.ZIP_DEFLATED,compresslevel=1) as archive:
        for name,raw in sorted(payload.items()):archive.writestr(name,raw)
        archive.writestr('r388/source_manifest.json',manifest_path.read_bytes())
    with zipfile.ZipFile(destination) as archive:
        assert all(digest(archive.read(n))==h for n,h in inventory.items())
    receipt=dict(prepared_only=True,files=len(inventory),frozen_source_count=len(frozen),staged_files=len(stage),
        archive_bytes=destination.stat().st_size,archive_sha256=digest(destination.read_bytes()),
        manifest_sha256=digest(manifest_path.read_bytes()),bootstrap_sha256=digest((A/'execute_isolated.py').read_bytes()),
        controller_calls=0,tests_executed=False,native_built=False,devices_opened=False)
    (A/'bundle_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8');print(json.dumps(receipt))


if __name__=='__main__':main()
