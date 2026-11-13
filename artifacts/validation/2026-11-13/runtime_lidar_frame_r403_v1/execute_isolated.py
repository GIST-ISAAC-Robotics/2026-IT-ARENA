"""지정 실행자만 사용. 새 디렉터리의 동결 소스와 ROS 메시지 단위검사만 실행."""
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


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root, receipt = args.root.resolve(), args.receipt.resolve()
    assert sys.platform == 'linux' and os.uname().machine == 'x86_64'
    assert root.parent == Path.home() and root.name.startswith('it-arena-r403-frame-') and not root.exists()
    assert receipt.parent == here and not receipt.exists()
    manifest = json.loads((here/'manifest.json').read_bytes())
    manifest_hash = digest(here/'manifest.json')
    def preparation_changes():
        changed = [name for name,sha in manifest['files'].items() if not (here/name).is_file() or digest(here/name)!=sha]
        if digest(here/'manifest.json') != manifest_hash: changed.append('manifest.json')
        return changed
    assert not preparation_changes()
    root.mkdir(); receipt.mkdir()
    with zipfile.ZipFile(here/'bundle_sources.zip') as archive:
        for name in archive.namelist():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
        archive.extractall(root)
    frozen = json.loads((root/'bundle_manifest.json').read_bytes())['files']
    def source_changes():
        return [name for name,sha in frozen.items() if not (root/name).is_file() or digest(root/name)!=sha]
    assert not source_changes()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        PYTHONPATH=os.pathsep.join(str(p) for p in (root/'src/arena_autonomy',
            root/'src/arena_vehicle_interface',root/'tests',root/'scripts')))
    command = [sys.executable,'-B',str(root/'verify_tests.py'),'--output',str(root/'results')]
    process = None; error = None; forced = False; returncode = None
    try:
        with (root/'tests.log').open('x') as log:
            process = subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try: returncode = process.wait(timeout=180)
            except subprocess.TimeoutExpired:
                forced = True; os.killpg(process.pid,signal.SIGTERM)
                try: returncode = process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); returncode = process.wait(timeout=5)
                raise
            if returncode: raise RuntimeError(('tests',returncode))
    except Exception as exc:
        error = repr(exc)
    finally:
        changed = source_changes(); preparation_changed = preparation_changes()
        result = dict(execution_completed=error is None and not changed and not preparation_changed,
            error=error,command=command,pid=None if process is None else process.pid,
            timeout_s=180,forced=forced,returncode=returncode,root=str(root),
            preparation_manifest_sha256=manifest_hash,source_changes=changed,
            preparation_changes=preparation_changed,source_files=len(frozen),
            physical_device_opened=False,ros_nodes_started=False,simulation_started=False)
        (root/'execution.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
        shutil.copyfile(root/'execution.json',receipt/'execution.json')
        with zipfile.ZipFile(receipt/'results.zip','x',zipfile.ZIP_DEFLATED) as archive:
            for path in [root/'execution.json',root/'tests.log',*(root/'results').rglob('*')]:
                if path.is_file(): archive.write(path,path.relative_to(root).as_posix())
        print(json.dumps(result))
    if error or changed or preparation_changed: raise SystemExit(1)


if __name__ == '__main__': main()
