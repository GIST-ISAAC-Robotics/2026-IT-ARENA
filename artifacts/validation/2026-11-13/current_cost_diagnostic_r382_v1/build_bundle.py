"""r380 동결 archive 원문+새 진단 도구만 묶는다. 현재 공통 src를 읽어 혼합하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile

from support import A, ROOT, digest, CHAIN


def main():
    old = A.parent/'current_cost_r380_v1'
    receipt = json.loads((old/'bundle_receipt.json').read_bytes())
    archive = A/'bundle_sources.zip'
    manifest_path = A/'source_manifest.json'
    assert not archive.exists() and not manifest_path.exists()
    assert digest(old/'bundle_sources.zip') == receipt['archive_sha256']
    old_manifest_bytes = (old/'source_manifest.json').read_bytes()
    assert hashlib.sha256(old_manifest_bytes).hexdigest() == receipt['manifest_sha256']
    old_manifest = json.loads(old_manifest_bytes)
    payload = {}
    with zipfile.ZipFile(old/'bundle_sources.zip') as zipped:
        for name, value in old_manifest['files'].items():
            content = zipped.read(name)
            assert hashlib.sha256(content).hexdigest() == value, ('frozen input mismatch',name)
            payload[name] = content
        old_name = (old/'source_manifest.json').relative_to(ROOT).as_posix()
        assert zipped.read(old_name) == old_manifest_bytes
        payload[old_name] = old_manifest_bytes
    assert len(old_manifest['files']) == 266
    for path in A.iterdir():
        if path.is_file() and (path.suffix in ('.py','.md') or path.name == 'original_diff.json'):
            name = path.relative_to(ROOT).as_posix()
            assert name not in payload
            payload[name] = path.read_bytes()
    for name, content in payload.items():
        if name.endswith('.py'): ast.parse(content, filename=name)
    inventory = {name:hashlib.sha256(content).hexdigest() for name,content in sorted(payload.items())}
    assert all(name in inventory for name in CHAIN)
    manifest = dict(schema='r382_frozen_r380_plus_diagnostic_v1', files=inventory,
        frozen_r380_archive_sha256=receipt['archive_sha256'], frozen_r380_manifest_sha256=receipt['manifest_sha256'],
        frozen_source_file_count=266, frozen_sources_identical=True, explicit_dynamic_chain=list(CHAIN),
        native_binary_included=False, current_workspace_common_sources_collected=False,
        original_r380_failure_preserved=True)
    with manifest_path.open('x', encoding='utf8') as stream:
        json.dump(manifest,stream,ensure_ascii=False,indent=2);stream.write('\n')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED,compresslevel=1) as zipped:
        for name,content in sorted(payload.items()):zipped.writestr(name,content)
        zipped.write(manifest_path,manifest_path.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(archive) as zipped:
        assert all(hashlib.sha256(zipped.read(n)).hexdigest()==h for n,h in inventory.items())
    result = dict(prepared_only=True,files=len(inventory),archive_bytes=archive.stat().st_size,
        archive_sha256=digest(archive),manifest_sha256=digest(manifest_path),
        bootstrap_sha256=digest(A/'remote_bootstrap.py'),frozen_r380_archive_sha256=receipt['archive_sha256'],
        original_266_file_bytes_preserved=True,jetson_accessed=False,native_built=False,timed_rebase_calls=0)
    (A/'bundle_receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result))


if __name__ == '__main__':main()
