"""동결 소스의 새 WSL root에서 검사 및223개 비교1회. 실제 ROS/주행 없음."""
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


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--receipt',type=Path,required=True);args=parser.parse_args()
    here=Path(__file__).resolve().parent;root=args.root.resolve();receipt=args.receipt.resolve()
    assert sys.platform=='linux' and os.uname().machine=='x86_64'
    assert root.parent==Path.home() and root.name.startswith('it-arena-r405-corner-') and not root.exists()
    assert receipt.parent==here and not receipt.exists()
    manifest=json.loads((here/'manifest.json').read_bytes());manifest_sha=digest(here/'manifest.json')
    def preparation_changes():
        result=[n for n,h in manifest['files'].items() if not (here/n).is_file() or digest(here/n)!=h]
        if digest(here/'manifest.json')!=manifest_sha:result.append('manifest.json')
        return result
    assert not preparation_changes();root.mkdir();receipt.mkdir()
    with zipfile.ZipFile(here/'bundle_sources.zip') as z:
        for name in z.namelist():assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
        z.extractall(root)
    frozen=json.loads((root/'bundle_manifest.json').read_bytes())['files']
    def changed():return [n for n,h in frozen.items() if not (root/n).is_file() or digest(root/n)!=h]
    assert not changed()
    isolated_paths=[str(p) for p in (root/'src/arena_autonomy',root/'src/arena_vehicle_interface',root/'tests',root/'scripts')]
    inherited_paths=[p for p in os.environ.get('PYTHONPATH','').split(os.pathsep) if p]
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        PYTHONPATH=os.pathsep.join(isolated_paths+inherited_paths))
    steps=[];error=None;native={}
    def execute(label,command,cwd,timeout):
        with (root/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            item=dict(label=label,command=command,pid=child.pid,timeout_s=timeout,forced=False);steps.append(item)
            try:item['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                item['forced']=True;os.killpg(child.pid,signal.SIGTERM)
                try:item['returncode']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);item['returncode']=child.wait(timeout=5)
                raise
            if item['returncode']:raise RuntimeError((label,item['returncode']))
    try:
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'src/arena_autonomy',120)
        binaries=list((root/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'))
        assert len(binaries)==1;native={p.relative_to(root).as_posix():digest(p) for p in binaries}
        execute('tests',[sys.executable,'-B','verify_tests.py'],root,120)
        execute('compare',[sys.executable,'-B','compare_current.py','--output',str(root/'results')],root,300)
    except Exception as exc:error=repr(exc)
    finally:
        source_changes=changed();prep_changes=preparation_changes()
        result=dict(execution_completed=error is None and not source_changes and not prep_changes,error=error,
            steps=steps,root=str(root),manifest_sha256=manifest_sha,source_changes=source_changes,
            preparation_changes=prep_changes,source_files=len(frozen),native_binaries=native,
            pythonpath=dict(isolated_first=isolated_paths,inherited_after=inherited_paths),
            simulation_started=False,physical_devices_opened=False,ros_nodes_started=False)
        (root/'execution.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
        shutil.copyfile(root/'execution.json',receipt/'execution.json')
        with zipfile.ZipFile(receipt/'results.zip','x',zipfile.ZIP_DEFLATED) as z:
            for path in [root/'execution.json',root/'tests_report.json',*root.glob('*.log'),*(root/'results').rglob('*')]:
                if path.is_file():z.write(path,path.relative_to(root).as_posix())
        print(json.dumps(result))
    if error or source_changes or prep_changes:raise SystemExit(1)


if __name__=='__main__':main()
