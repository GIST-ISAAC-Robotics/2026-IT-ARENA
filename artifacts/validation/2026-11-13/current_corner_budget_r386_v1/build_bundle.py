"""r377 실행 당시 원문을 수집해 r386 격리 번들을 만든다. 새 제어 계산/빌드 없음."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile

from support import A,ROOT,CHAIN,digest


def main():
    old=A.parent/'released_cad_drive_r377_v1'
    registered=json.loads((old/'registered.json').read_bytes())
    assert digest(old/'sources.zip')==registered['source_archive_sha256']
    destination=A/'bundle_sources.zip';manifest_path=A/'source_manifest.json'
    assert not destination.exists() and not manifest_path.exists(),'preserve prior bundle'
    payload={};frozen={}
    with zipfile.ZipFile(old/'sources.zip') as zipped:
        for name in zipped.namelist():
            wanted=(name.startswith(('src/arena_autonomy/','src/arena_vehicle_interface/'))
                or name.startswith('artifacts/validation/2026-11-12/retained_route_execution_r266_v1/')
                or name in CHAIN or name=='artifacts/validation/2026-11-13/released_current_r375_v1/compare.py')
            if not wanted or Path(name).suffix not in ('.py','.cpp','.hpp','.h','.xml','.cfg'):continue
            content=zipped.read(name);h=hashlib.sha256(content).hexdigest()
            assert h==registered['input_sha256'][name],('original source mismatch',name)
            payload[name]=content;frozen[name]=h
    assert all(name in payload for name in CHAIN)
    comparison=A.parent/'released_current_r375_v1/results_native_v1/comparison.json'
    old_comparison=json.loads(comparison.read_bytes())
    inputs={ROOT/name:h for name,h in old_comparison['input_hashes'].items()}
    audit=json.loads((old/'audit.json').read_bytes())
    trace=old/'run1/follow_decisions.jsonl'
    # 원 r377 audit의 저장 trace 해시도 일치해야 한다.
    trace_key=next(k for k in audit['input_sha256'] if k.endswith('follow_decisions.jsonl'))
    assert digest(trace)==audit['input_sha256'][trace_key]
    inputs[trace]=digest(trace)
    for path,h in inputs.items():
        assert digest(path)==h
        payload[path.relative_to(ROOT).as_posix()]=path.read_bytes()
    for path in (old/'registered.json',old/'audit.json',old/'run1/report.json',comparison):
        payload[path.relative_to(ROOT).as_posix()]=path.read_bytes()
    for path in A.iterdir():
        if path.is_file() and path.suffix in ('.py','.md'):
            payload[path.relative_to(ROOT).as_posix()]=path.read_bytes()
    for name,content in payload.items():
        if name.endswith('.py'):ast.parse(content,filename=name)
    inventory={name:hashlib.sha256(content).hexdigest() for name,content in sorted(payload.items())}
    manifest=dict(schema='r386_r377_frozen_saved_input_bundle',files=inventory,
        source_r377_archive_sha256=registered['source_archive_sha256'],frozen_r377_source_hashes=frozen,
        inputs={path.relative_to(ROOT).as_posix():h for path,h in inputs.items()},explicit_dynamic_chain=list(CHAIN),
        current_common_source_collected=False,native_binary_included=False,original_failure_preserved=True)
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(destination,'x',zipfile.ZIP_DEFLATED,compresslevel=1) as zipped:
        for name,content in sorted(payload.items()):zipped.writestr(name,content)
        zipped.write(manifest_path,manifest_path.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(destination) as zipped:
        assert all(hashlib.sha256(zipped.read(name)).hexdigest()==h for name,h in inventory.items())
    receipt=dict(prepared_only=True,files=len(inventory),frozen_r377_sources=len(frozen),archive_bytes=destination.stat().st_size,
        archive_sha256=digest(destination),manifest_sha256=digest(manifest_path),
        native_built=False,candidate_calls=0,tests_executed=False,devices_opened=False)
    (A/'bundle_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8')
    print(json.dumps(receipt))


if __name__=='__main__':main()
