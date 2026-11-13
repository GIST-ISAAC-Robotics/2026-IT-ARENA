"""보존 r386 입력/알고리즘과 승인 r388 r1 소스 arm을 묶는다. 계산·실행 없음."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;P=A.parent
R386='4a116fd85b60a9ece7bb9dbdeb197aa872314f595b7219ed9b0b3dafe025ea04'
R388='f0d6b8496b02a41686adef66f62114e21b98c77d928cc9e780f8a35524a25a40'


def digest(raw):return hashlib.sha256(raw).hexdigest()


def main():
    archive_path=A/'bundle_sources.zip';manifest_path=A/'source_manifest.json'
    assert not archive_path.exists() and not manifest_path.exists(),'preserve prepared revision'
    original=P/'bundle_sources.zip';geometry=P.parent/'final_geometry_contract_r388_v1/revision_r1/bundle_sources.zip'
    assert digest(original.read_bytes())==R386 and digest(geometry.read_bytes())==R388
    r388_result=geometry.parent/'remote_result/extracted/results_native_v1/comparison.json'
    assert json.loads(r388_result.read_bytes())['passed'] is True
    payload={};algorithm={}
    with zipfile.ZipFile(original) as archive:
        old_path='artifacts/validation/2026-11-13/current_corner_budget_r386_v1/'
        old=json.loads(archive.read(old_path+'source_manifest.json'))
        assert all(digest(archive.read(n))==h for n,h in old['files'].items())
        for name in old['inputs']:payload['inputs/'+name]=archive.read(name)
        for suffix in ('compare.py','candidate.py','budget.py','test_candidate.py','source_manifest.json'):
            payload['inputs/'+old_path+suffix]=archive.read(old_path+suffix)
        for name in ('candidate.py','budget.py','test_candidate.py'):
            raw=(A/name).read_bytes();assert raw==archive.read(old_path+name),(name,'original algorithm/test changed')
            algorithm[name]=digest(raw)
    with zipfile.ZipFile(geometry) as archive:
        selected=json.loads(archive.read('r388/source_manifest.json'))
        assert all(digest(archive.read(n))==h for n,h in selected['files'].items())
        for name in archive.namelist():
            if name.startswith(('frozen/','geometry/')) and not name.startswith('geometry/tests/'):
                payload[name]=archive.read(name)
        for name,h in old['frozen_r377_source_hashes'].items():assert digest(payload['frozen/'+name])==h
    test=A/'staged_tests/test_vehicle_geometry_contract.py'
    payload['geometry/tests/test_vehicle_geometry_contract.py']=test.read_bytes()
    for path in A.iterdir():
        if path.is_file() and path.suffix in ('.py','.md'):payload['r386/'+path.name]=path.read_bytes()
    payload['r386/staged_tests/test_vehicle_geometry_contract.py']=test.read_bytes()
    for name,raw in payload.items():
        if name.endswith('.py'):ast.parse(raw,filename=name)
    inventory={name:digest(raw) for name,raw in sorted(payload.items())}
    manifest=dict(schema='r386_final_geometry_causal_comparison',files=inventory,original_r386_archive_sha256=R386,
        approved_r388_r1_archive_sha256=R388,approved_r388_result_sha256=digest(r388_result.read_bytes()),
        original_r386_algorithm_sha256=algorithm,frozen_source_count=selected['frozen_source_count'],
        geometry_transplants=selected['geometry_only_transplants'],candidate_chain=selected['candidate_chain'],
        input_hashes=old['inputs'],raw_inputs_copied_once=True,algorithm_modified=False,
        current_unrelated_source_collected=False,original_r386_archive_executed=False)
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(archive_path,'x',zipfile.ZIP_DEFLATED,compresslevel=1) as archive:
        for name,raw in sorted(payload.items()):archive.writestr(name,raw)
        archive.writestr('r386/source_manifest.json',manifest_path.read_bytes())
    with zipfile.ZipFile(archive_path) as archive:
        assert all(digest(archive.read(name))==h for name,h in inventory.items())
    receipt=dict(prepared_only=True,files=len(inventory),archive_bytes=archive_path.stat().st_size,
        archive_sha256=digest(archive_path.read_bytes()),manifest_sha256=digest(manifest_path.read_bytes()),
        bootstrap_sha256=digest((A/'execute_isolated.py').read_bytes()),algorithm_sha256=algorithm,
        controller_calls=0,tests_executed=False,native_built=False,devices_opened=False)
    (A/'bundle_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8');print(json.dumps(receipt))


if __name__=='__main__':main()
