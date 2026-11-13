"""r493 원본을 쓰지 않고 r497 소유 계산 한 번을 최대 240초 안에 회수한다."""
import hashlib
import json
import os
import signal
import subprocess
import sys
import traceback
import zipfile
from pathlib import Path

A = Path(__file__).resolve().parent
SOURCE = Path('/home/q/it-arena-r493-a')
sys.path.insert(0, str(SOURCE))
from environment_reference import other_work, safe_environment_snapshot
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert A == Path('/home/q/it-arena-r497-a') and os.uname().machine == 'aarch64'
    selection = json.loads((A/'selection.json').read_bytes())
    source_manifest = SOURCE/'source_manifest.json'
    assert sha(source_manifest) == selection['r493_manifest_sha256']
    original = json.loads(source_manifest.read_bytes())
    for path, digest in selection['helper_files'].items():
        assert sha(A/path) == digest, path
    native = SOURCE/'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so'
    assert sha(native) == selection['native_sha256']
    assert sha(SOURCE/'results/outputs.jsonl') == selection['r493_outputs_sha256']
    def changed():
        return [p for p,h in original['files'].items() if sha(SOURCE/p) != h]
    assert not changed()
    before_work = other_work(A)
    assert not any(before_work.values()), before_work
    assert not (A/'execution_receipt.json').exists() and not (A/'run.pid').exists()
    (A/'results').mkdir()
    before = safe_environment_snapshot()
    step, error = {}, None
    try:
        with (A/'profile.log').open('x') as log:
            child = subprocess.Popen([sys.executable,'-B',str(A/'profile_calls.py')], cwd=A,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'), stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            step = dict(pid=child.pid, timeout_s=240, forced=False)
            (A/'run.pid').write_text(str(child.pid))
            print(json.dumps(dict(started=True, **step)), flush=True)
            try:
                step['exit_code'] = child.wait(timeout=240)
            except subprocess.TimeoutExpired:
                step['forced'] = True
                os.killpg(child.pid,signal.SIGINT)
                try: step['exit_code'] = child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL)
                    step['exit_code'] = child.wait(timeout=5)
            try: os.killpg(child.pid,0)
            except ProcessLookupError: step['residual_group'] = False
            else:
                step['residual_group'] = True
                os.killpg(child.pid,signal.SIGKILL)
            assert step['exit_code']==0 and not step['forced'] and not step['residual_group'], step
    except BaseException as exc:
        error = repr(exc)
        (A/'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt = dict(completed=error is None, error=error, step=step, source_changes=changed(),
            r493_manifest_unchanged=sha(source_manifest)==selection['r493_manifest_sha256'],
            r493_outputs_unchanged=sha(SOURCE/'results/outputs.jsonl')==selection['r493_outputs_sha256'],
            r493_native_unchanged=sha(native)==selection['native_sha256'],
            environment_before=before, environment_after=safe_environment_snapshot(),
            preflight_work=before_work, remaining=other_work(A), ports_opened=False,
            native_build=False, packages_installed=False, settings_changed=False, ros_started=False,
            driving=False, state_injected_per_row=False, throughput_claim=False)
        (A/'execution_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        archive = A.with_name(A.name+'-results.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in [A/'execution_receipt.json', A/'traceback.txt', A/'profile.log', *(A/'results').glob('*')]:
                if p.is_file(): z.write(p,p.relative_to(A).as_posix())
        print(json.dumps(dict(completed=receipt['completed'],error=error,step=step,
            source_changes=receipt['source_changes'],remaining=receipt['remaining'],
            archive=str(archive),archive_sha256=sha(archive))),flush=True)
    return 0 if receipt['completed'] and not receipt['source_changes'] and not any(receipt['remaining'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
