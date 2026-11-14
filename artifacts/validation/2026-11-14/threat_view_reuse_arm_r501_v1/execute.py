"""root가 Jetson 소유권을 넘긴 뒤 실행하는 r501 전용 비구동 순차 비교."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import traceback
import zipfile

from environment_reference import other_work, safe_environment_snapshot

A = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resource-owner', choices=('r501',), required=True)
    parser.parse_args()
    assert A == Path('/home/q/it-arena-r501-a') and sys.platform == 'linux' and os.uname().machine == 'aarch64'
    manifest_path = A/'source_manifest.json'
    manifest = json.loads(manifest_path.read_bytes())
    manifest_sha = sha(manifest_path)
    assert not (A/'execution_receipt.json').exists() and not (A/'results').exists()
    preflight = other_work(A)
    assert not any(preflight.values()), preflight
    changes = lambda: [p for p, h in manifest['files'].items() if not (A/p).is_file() or sha(A/p) != h]
    assert not changes()
    reuse = manifest['native_reuse']
    native = Path(reuse['source'])
    assert sha(native) == reuse['sha256']
    original_source = native.parents[3]
    for path, digest in reuse['source_sha256'].items():
        assert sha(original_source/path) == digest, path
        for variant in ('baseline', 'candidate'):
            assert sha(A/variant/path) == digest, (variant, path)
    copied = []
    for variant in ('baseline', 'candidate'):
        destination = A/variant/'src/arena_autonomy/arena_autonomy'/native.name
        assert not destination.exists()
        shutil.copyfile(native, destination)
        copied.append(destination)
    (A/'results').mkdir()
    steps, failure = [], None
    before = safe_environment_snapshot()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    # 이 비교의 import는 각 자식의 동결 경로만 지정한다. 사용자의 셸 설정은 변경하지 않는다.
    env.pop('PYTHONPATH', None)
    def run(label, command, timeout):
        with (A/(label+'.log')).open('x') as log:
            child = subprocess.Popen(command, cwd=A, env=env, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
            step = dict(label=label, command=command, pid=child.pid, timeout_s=timeout, forced=False)
            steps.append(step)
            (A/(label+'.pid')).write_text(str(child.pid))
            try:
                step['exit_code'] = child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                step['forced'] = True
                os.killpg(child.pid, signal.SIGINT)
                try:
                    step['exit_code'] = child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    step['exit_code'] = child.wait(timeout=5)
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                step['residual_group'] = False
            else:
                step['residual_group'] = True
                os.killpg(child.pid, signal.SIGKILL)
            assert step['exit_code'] == 0 and not step['forced'] and not step['residual_group'], step
    try:
        for variant in ('baseline', 'candidate'):
            run(variant, [sys.executable, '-B', str(A/'run_variant.py'), variant], 240)
        run('compare', [sys.executable, '-B', str(A/'compare.py')], 90)
    except BaseException as exc:
        failure = repr(exc)
        (A/'traceback.txt').write_text(traceback.format_exc())
    finally:
        source_changes = changes()
        remaining = other_work(A)
        manifest_unchanged = sha(manifest_path) == manifest_sha
        native_original_unchanged = sha(native) == reuse['sha256']
        native_copies = {str(p.relative_to(A)):sha(p) for p in copied}
        receipt = dict(completed=(failure is None and not source_changes and not any(remaining.values())
                and manifest_unchanged and native_original_unchanged
                and all(h == reuse['sha256'] for h in native_copies.values())),
            error=failure, steps=steps, source_changes=source_changes,
            manifest_unchanged=manifest_unchanged,
            native_original_unchanged=native_original_unchanged,
            native_copies=native_copies,
            native_source_hash_verified=True, native_build=False, preflight_work=preflight, remaining=remaining,
            environment_before=before, environment_after=safe_environment_snapshot(),
            ports_opened=False, settings_changed=False, packages_installed=False,
            ros_started=False, driving=False, physical_hardware_verified=False)
        (A/'execution_receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
        archive = A.with_name(A.name+'-results.zip')
        with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
            for path in [A/'execution_receipt.json', A/'traceback.txt', *A.glob('*.log'), *A.glob('*.pid'), *(A/'results').rglob('*')]:
                if path.is_file():
                    output.write(path, path.relative_to(A).as_posix())
        print(json.dumps(dict(completed=receipt['completed'], error=failure, steps=steps,
            source_changes=source_changes, remaining=remaining, archive=str(archive), archive_sha256=sha(archive))), flush=True)
    return 0 if receipt['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
