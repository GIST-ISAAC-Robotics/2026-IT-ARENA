"""r467 자원 배정 안에서 두 계산 프로세스만 실행하고 PID/입력을 회수한다."""
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

ROOT = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert os.uname().machine == 'aarch64' and ROOT == Path('/home/q/it-arena-r467-a')
    manifest_path = ROOT / 'source_manifest.json'
    manifest = json.loads(manifest_path.read_bytes())
    manifest_hash = sha(manifest_path)
    assert manifest['root_reviewed'] and manifest['jetson_exclusive_resource_assigned']
    assert manifest['computations'] == 24 and manifest['warmups'] == 2
    assert not (ROOT / 'execution_receipt.json').exists()
    interference = other_work(ROOT)
    assert not any(interference.values()), interference
    def changed():
        found = [path for path, digest in manifest['files'].items()
                 if not (ROOT / path).is_file() or sha(ROOT / path) != digest]
        if sha(manifest_path) != manifest_hash:
            found.append('source_manifest.json')
        return found
    assert not changed()
    native = manifest['native_reuse']
    native_source = Path(native['source'])
    assert sha(native_source) == native['sha256']
    for version in ('base', 'stage'):
        for path, digest in native['original_source_sha256'].items():
            assert sha(ROOT / version / path) == digest
        destination = ROOT / version / 'src/arena_autonomy/arena_autonomy' / native_source.name
        assert not destination.exists()
        shutil.copyfile(native_source, destination)
        assert sha(destination) == native['sha256']
    (ROOT / 'results').mkdir(exist_ok=False)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=str(ROOT))
    before = safe_environment_snapshot()
    steps, error = [], None
    try:
        for version in ('base', 'stage'):
            command = [sys.executable, '-B', str(ROOT / 'run_cost.py'), version]
            with (ROOT / (version + '.log')).open('x') as log:
                child = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                row = dict(version=version, command=command, pid=child.pid, timeout_s=60, forced=False)
                steps.append(row)
                try:
                    row['returncode'] = child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    row['forced'] = True
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        row['returncode'] = child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        row['returncode'] = child.wait(timeout=5)
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    row['residual_group'] = False
                else:
                    row['residual_group'] = True
                    os.killpg(child.pid, signal.SIGKILL)
                assert row['returncode'] == 0 and not row['forced'] and not row['residual_group'], row
    except BaseException as exc:
        error = repr(exc)
        (ROOT / 'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt = dict(completed=error is None, error=error, steps=steps,
            source_changes=changed(), source_manifest_sha256=manifest_hash,
            native_sha256=sha(native_source), native_reused=True, new_build=False,
            environment_before=before, environment_after=safe_environment_snapshot(),
            preflight_interference=interference, remaining=other_work(ROOT),
            hardware_activation=False, physical_measurement=False, ports_opened=False)
        (ROOT / 'execution_receipt.json').write_text(json.dumps(receipt, indent=2))
        archive = ROOT.with_name(ROOT.name + '-results.zip')
        with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
            for path in [ROOT / 'execution_receipt.json', ROOT / 'traceback.txt',
                         *ROOT.glob('*.log'), *(ROOT / 'results').glob('*.json')]:
                if path.is_file():
                    z.write(path, path.relative_to(ROOT).as_posix())
        print(json.dumps(dict(completed=receipt['completed'], error=error, steps=steps,
            source_changes=receipt['source_changes'], remaining=receipt['remaining'],
            archive=str(archive), archive_sha256=sha(archive))))
    return 0 if error is None and not receipt['source_changes'] and not any(receipt['remaining'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
