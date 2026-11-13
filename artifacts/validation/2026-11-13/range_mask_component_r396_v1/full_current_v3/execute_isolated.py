"""지정 WSL 실행 담당자용. 원 r386 archive는 읽고 새 r396 root에서만 작업한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import zipfile

ARCHIVE_SHA256 = 'bf53444bbed329082c904d046921656e0035f9c1715c18e6e11812fb48cbfd52'


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--r386-archive', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root, receipt = args.root.resolve(), args.receipt.resolve()
    assert sys.platform == 'linux' and os.uname().machine == 'x86_64', 'assigned WSL x86 executor only'
    assert root.is_relative_to(Path.home()) and root.name.startswith('it-arena-r396-full-') and not root.exists()
    assert receipt.parent == here and not receipt.exists(), 'fresh receipt inside r396 artifact required'
    archive = args.r386_archive.resolve()
    assert digest(archive) == ARCHIVE_SHA256
    manifest = json.loads((here/'manifest.json').read_bytes())
    for name, sha in manifest['files'].items(): assert digest(here/name) == sha, ('r396 changed', name)
    root.mkdir(); receipt.mkdir()
    with zipfile.ZipFile(archive) as z:
        for name in z.namelist():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
        z.extractall(root)
    work = root/'r396'; work.mkdir()
    for name in manifest['files']:
        target = work/name; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(here/name, target)
    frozen = json.loads((root/'r386/source_manifest.json').read_bytes())['files']
    def changed():
        result = [name for name, sha in frozen.items() if not (root/name).is_file() or digest(root/name) != sha]
        result += ['r396/'+name for name, sha in manifest['files'].items() if digest(work/name) != sha]
        result += ['original_r396/'+name for name, sha in manifest['files'].items() if digest(here/name) != sha]
        return result
    assert not changed()
    source = root/'geometry'
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        PYTHONPATH=os.pathsep.join(str(p) for p in (work/'staged', work, root/'r386',
            source/'src/arena_autonomy', source/'src/arena_vehicle_interface', source/'scripts')),
        R386_SOURCE_ROOT=str(source), R386_INPUT_ROOT=str(root/'inputs'))
    steps = []; error = None; binaries = {}
    def execute(label, command, cwd, timeout):
        with (work/(label+'.log')).open('x') as log:
            child = subprocess.Popen(command, cwd=cwd, env=environment, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            step = dict(label=label, command=command, pid=child.pid, timeout_s=timeout, forced=False)
            steps.append(step)
            try: step['returncode'] = child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                step['forced'] = True; os.killpg(child.pid, signal.SIGTERM)
                try: step['returncode'] = child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL); step['returncode'] = child.wait(timeout=5)
                raise
            if step['returncode']: raise RuntimeError((label, step['returncode']))
    try:
        execute('native_build', [sys.executable, '-B', 'setup.py', 'build_ext', '--inplace'],
                source/'src/arena_autonomy', 120)
        native = list((source/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'))
        assert len(native) == 1
        binaries = {str(native[0].relative_to(root)): digest(native[0])}
        execute('compare_current', [sys.executable, '-B', str(work/'compare_current.py'), '--output',
                                  str(work/'results')], work, 300)
    except Exception as exc:
        error = repr(exc)
    finally:
        changes = changed()
        result = dict(execution_completed=error is None and not changes, error=error, steps=steps,
            source_changes=changes, r386_source_files=len(frozen), native_binaries=binaries,
            root=str(root), archive_sha256=ARCHIVE_SHA256,
            r396_manifest_sha256=digest(here/'manifest.json'), original_artifacts_overwritten=False,
            devices_opened=False, simulation_started=False, new_default=False,
            unit29_repeated=False, original795_replayed=False, legacy795_replayed=False)
        (work/'execution.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
        shutil.copyfile(work/'execution.json', receipt/'execution.json')
        with zipfile.ZipFile(receipt/'results.zip', 'x', zipfile.ZIP_DEFLATED) as z:
            for path in [*work.glob('*.log'), work/'execution.json', *(work/'results').rglob('*')]:
                if path.is_file(): z.write(path, path.relative_to(work).as_posix())
        print(json.dumps(result))
    if error or changes: raise SystemExit(1)


if __name__ == '__main__': main()
