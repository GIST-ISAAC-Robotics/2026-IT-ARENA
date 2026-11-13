"""첫 실패를 보존하고 시험용 G1 필수 설정만 고친 새 동결 번들을 만든다."""
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
PARENT = HERE.parent


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def write(name, raw):
    with (HERE/name).open('xb') as stream:
        stream.write(raw)


def main():
    original = json.loads((PARENT/'manifest.json').read_bytes())
    assert sha((PARENT/'manifest.json').read_bytes()) == 'dc04d74b594c603087394da2e90b7bd8e57ebd749bf0dac726858fc95daf94bd'
    assert all(sha((PARENT/name).read_bytes()) == value for name, value in original['files'].items())
    with zipfile.ZipFile(PARENT/'bundle_sources.zip') as archive:
        members = {name: archive.read(name) for name in archive.namelist()}
    manifest = json.loads(members['bundle_manifest.json'])
    name = 'tests/test_current_corner_runtime.py'
    old = members[name].decode('utf8')
    assert old.count('follow=NS(config=FollowConfig()),motion=') == 1
    revised = old.replace('from arena_autonomy.bypass_handoff import HandoffExecutionConfig',
                         'from arena_autonomy.bypass_handoff import HandoffExecutionConfig, handoff_baseline')
    revised = revised.replace('follow=NS(config=FollowConfig()),motion=',
                              'follow=NS(config=handoff_baseline()),motion=')
    members[name] = revised.encode('utf8')
    manifest['files'][name] = sha(members[name])
    members['bundle_manifest.json'] = (json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode('utf8')
    write('test_current_corner_runtime.py', members[name])
    write('bundle_manifest.json', members['bundle_manifest.json'])
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name, raw in sorted(members.items()):
            archive.writestr(name,raw)
    write('execute_isolated.py', (PARENT/'execute_isolated.py').read_bytes())
    changed = [name for name,value in manifest['files'].items()
               if json.loads((PARENT/'bundle_manifest.json').read_bytes())['files'][name] != value]
    assert changed == ['tests/test_current_corner_runtime.py']
    write('REVISION.json', (json.dumps(dict(changed_source_files=changed,algorithm_unchanged=True,
        old_manifest_sha256=sha((PARENT/'manifest.json').read_bytes()),
        original_failure='../verify_v1/extracted/tests.log',
        cause='node-method fixture incorrectly supplied bare FollowConfig instead of required handoff baseline'),
        ensure_ascii=False,indent=2)+'\n').encode('utf8'))
    outer = dict(schema='r405_fixture_revision_v2',files={
        p.name:sha(p.read_bytes()) for p in sorted(HERE.iterdir()) if p.is_file()})
    write('manifest.json',(json.dumps(outer,indent=2)+'\n').encode('utf8'))
    print(json.dumps(dict(manifest_sha256=sha((HERE/'manifest.json').read_bytes()),
                         bundle_sha256=sha((HERE/'bundle_sources.zip').read_bytes()),changed=changed)))


if __name__ == '__main__':
    main()
