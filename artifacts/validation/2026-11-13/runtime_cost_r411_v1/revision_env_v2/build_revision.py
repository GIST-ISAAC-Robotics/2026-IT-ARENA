"""원 r411 archive157개 중 bootstrap 1개만 바꾼다. common/입력/기존 결과는 읽기 전용."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent
ORIGINAL=HERE.parent
def sha(data):return hashlib.sha256(data).hexdigest()
def output(path,data):
    with path.open('xb') as stream:stream.write(data)
def packed(value):return (json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf8')


def main():
    original_zip=ORIGINAL/'bundle_sources.zip'
    assert sha(original_zip.read_bytes())=='cfdca33516c9ff638866e6224fe0c17be425b157979254bc91b3c8c06c02a06d'
    old_preparation=json.loads((ORIGINAL/'preparation_manifest.json').read_bytes())
    assert all(sha((ORIGINAL/n).read_bytes())==h for n,h in old_preparation['files'].items())
    with zipfile.ZipFile(original_zip) as source:
        old_manifest_bytes=source.read('source_manifest.json');manifest=json.loads(old_manifest_bytes)
        members={n:source.read(n) for n in manifest['files']}
    assert len(members)==157 and all(sha(d)==manifest['files'][n] for n,d in members.items())
    key='r411/remote_bootstrap.py';old=members[key];new=(HERE/'remote_bootstrap.py').read_bytes()
    # main은 metadata snapshot 호출 두 곳만 달라야 한다.
    old_tree=ast.parse(old);new_tree=ast.parse(new)
    old_main=next(n for n in old_tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    new_main=next(n for n in new_tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    renamed=0
    for n in ast.walk(new_main):
        if isinstance(n,ast.Name) and n.id=='safe_environment_snapshot':n.id='environment_snapshot';renamed+=1
    assert renamed==2 and ast.dump(old_main)==ast.dump(new_main)
    members[key]=new
    changed=[n for n,d in members.items() if sha(d)!=manifest['files'][n]]
    assert changed==[key]
    manifest['files'][key]=sha(new)
    output(HERE/'source_manifest.json',packed(manifest))
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as target:
        for n,d in sorted(members.items()):target.writestr(n,d)
        target.writestr('source_manifest.json',packed(manifest))
    output(HERE/'bootstrap.patch',''.join(difflib.unified_diff(old.decode().splitlines(keepends=True),
        new.decode().splitlines(keepends=True),fromfile='original/r411/remote_bootstrap.py',
        tofile='revision_env_v2/r411/remote_bootstrap.py')).encode())
    reference=HERE/'reference';reference.mkdir()
    output(reference/'original_bootstrap.py',old)
    output(reference/'original_source_manifest.json',old_manifest_bytes)
    log=ORIGINAL/'remote_execution.log';output(reference/'original_remote_execution.log',log.read_bytes())
    smoke=ORIGINAL/'verify_smoke_v1/extracted/smoke/report.json';original_smoke=json.loads(smoke.read_bytes())
    assert original_smoke['passed'] is True and original_smoke['completed_calls']==80
    receipt=dict(schema='r411_environment_revision_v2',files=157,changed_original_files=changed,unchanged_original_files=156,
        archive_sha256=sha((HERE/'bundle_sources.zip').read_bytes()),archive_bytes=(HERE/'bundle_sources.zip').stat().st_size,
        manifest_sha256=sha(packed(manifest)),bootstrap_sha256=sha(new),
        original_archive_sha256=sha(original_zip.read_bytes()),original_manifest_sha256=sha(old_manifest_bytes),
        original_failure_log_sha256=sha(log.read_bytes()),original_smoke_report_sha256=sha(smoke.read_bytes()),
        original_smoke_passed=True,old_smoke_authorizes_revision=False,new_smoke_required=True,
        input_contract_sha256=manifest['input_contract_sha256'],main_delta_only_two_metadata_calls=True,
        control_input_timing_judgment_unchanged=True,new_execution_performed=False)
    output(HERE/'bundle_receipt.json',packed(receipt));print(json.dumps(receipt))


if __name__=='__main__':main()
