"""Windows에서 현재 공통/검토한 r498 후보를 동결한다. 원격 접근/원본 변경 없음."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PREVIOUS = A.parent/'threat_view_reuse_r498_v1'
B = A/'bundle'
CHANGED = ['src/arena_autonomy/arena_autonomy/'+name for name in
           ('local_follow.py', 'pass_release_trial.py', 'traffic_solo_gap.py')]
NEW_TEST = 'tests/test_threat_view_transfer.py'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original_manifest = json.loads((PREVIOUS/'source_manifest.json').read_bytes())
    original_bad = [p for p,h in original_manifest.items() if sha(PREVIOUS/'original'/p) != h]
    common_bad = [p for p,h in original_manifest.items() if sha(ROOT/p) != h]
    stage_changed = [p for p,h in original_manifest.items() if sha(PREVIOUS/'stage'/p) != h]
    stage_added = sorted(p.relative_to(PREVIOUS/'stage').as_posix() for p in (PREVIOUS/'stage').rglob('*.py')
        if p.relative_to(PREVIOUS/'stage').as_posix() not in original_manifest)
    assert not original_bad and not common_bad
    assert sorted(stage_changed) == sorted(CHANGED) and stage_added == [NEW_TEST]
    B.mkdir()
    paths = set(original_manifest)
    for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
        paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src'/pkg/pkg).rglob('*')
            if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h') and '__pycache__' not in p.parts)
    current = {p:sha(ROOT/p) for p in sorted(paths)}
    for variant in ('baseline', 'candidate'):
        for path in sorted(paths):
            source = PREVIOUS/'stage'/path if variant == 'candidate' and path in CHANGED else ROOT/path
            destination = B/variant/path
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
    test = B/'candidate'/NEW_TEST
    test.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(PREVIOUS/'stage'/NEW_TEST, test)
    input_path = A.parent/'jetson_lap_cost_r493_v1/bundle/inputs.json'
    assert sha(input_path) == '909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac'
    shutil.copyfile(input_path, B/'inputs.json')
    inputs = json.loads((B/'inputs.json').read_bytes())
    assert len(inputs['plans']) == 508 and inputs['header']['numeric_execution'] == 'native_v1'
    prior = A.parent/'jetson_lap_cost_r493_v1/bundle'
    reuse = json.loads((prior/'source_manifest.json').read_bytes())['native_reuse']
    for path, digest in reuse['source_sha256'].items():
        for variant in ('baseline','candidate'):
            assert sha(B/variant/path) == digest, (variant, path)
    for name in ('run_variant.py','compare.py','state_codec.py','execute.py'):
        shutil.copyfile(A/name, B/name)
    shutil.copyfile(prior/'environment_reference.py', B/'environment_reference.py')
    patch = []
    for path in CHANGED+[NEW_TEST]:
        old = (ROOT/path).read_text(encoding='utf8').splitlines(True) if (ROOT/path).exists() else []
        new = (PREVIOUS/'stage'/path).read_text(encoding='utf8').splitlines(True)
        patch.extend(difflib.unified_diff(old, new, fromfile='a/'+path if old else '/dev/null', tofile='b/'+path))
    (A/'change.patch').write_text(''.join(patch), encoding='utf8', newline='\n')
    assert not [p for p,h in current.items() if sha(ROOT/p) != h]
    files = {p.relative_to(B).as_posix():sha(p) for p in sorted(B.rglob('*')) if p.is_file()}
    head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    changed_hashes = {p:dict(baseline=sha(B/'baseline'/p) if (B/'baseline'/p).exists() else None,
        candidate=sha(B/'candidate'/p)) for p in CHANGED+[NEW_TEST]}
    manifest = dict(files=files, root_head=head, current_common_sources=current,
        source_scope='current common bytes at preparation, with only three reviewed r498 changes in candidate; no historical r493 policy source',
        changed_files=changed_hashes, native_reuse=reuse, plan_calls=508,
        input_sha256=sha(B/'inputs.json'), input_header_changed=False,
        original_r498_source_manifest_sha256=sha(PREVIOUS/'source_manifest.json'),
        remote_root='/home/q/it-arena-r501-a', requires_root_resource_handoff=True,
        execution_started=False, jetson_accessed=False, wsl_accessed=False,
        devices_opened=False, driving=False, settings_changed=False)
    (B/'source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf8')
    shutil.copyfile(B/'source_manifest.json', A/'source_manifest.json')
    with zipfile.ZipFile(A/'bundle.zip', 'x', zipfile.ZIP_DEFLATED) as output:
        for path in sorted(B.rglob('*')):
            if path.is_file():
                output.write(path, path.relative_to(B).as_posix())
    receipt = dict(prepared=True, frozen_files=len(files), source_files_each=len(paths),
        original_manifest_checked=len(original_manifest), original_mismatches=original_bad,
        common_mismatches=common_bad, stage_changed=stage_changed, stage_added=stage_added,
        current_common_sha256=current, changed_files=changed_hashes, input_sha256=sha(input_path),
        bundle_sha256=sha(A/'bundle.zip'), root_head=head, arm_execution_started=False,
        r498_source_modified=False, common_source_modified=False)
    (A/'prepared.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in receipt.items() if k != 'current_common_sha256'}, indent=2))


if __name__ == '__main__':
    main()
