"""새 WSL root의 동결 소스에 native build/import/실제 ROS+PTY 1회만 배정한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import zipfile


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--receipt',type=Path,required=True);args=parser.parse_args()
    here=Path(__file__).resolve().parent;root=args.root.resolve();receipt=args.receipt.resolve()
    assert sys.platform=='linux' and os.uname().machine=='x86_64'
    assert root.parent==Path.home() and root.name.startswith('it-arena-r407-ops-v2-') and not root.exists()
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
    isolated=[str(root/n) for n in ('src/arena_autonomy','src/arena_vehicle_interface','scripts','r407')]
    inherited=[p for p in os.environ.get('PYTHONPATH','').split(os.pathsep) if p]
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(isolated+inherited),
        ROS_DOMAIN_ID='219',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    steps=[];error=None;native={}
    def execute(label,command,cwd,timeout):
        with (root/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            item=dict(label=label,command=command,pid=child.pid,timeout_s=timeout,forced=False);steps.append(item)
            timed_out=False
            try:item['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out=True;item['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:item['returncode']=child.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);item['returncode']=child.wait(timeout=3.)
            # Parent exited but a child could still own the process group. Always audit it.
            end=time.monotonic()+.5
            while True:
                try:os.killpg(child.pid,0)
                except ProcessLookupError:item['residual_group']=False;break
                if time.monotonic()>=end:
                    item['residual_group']=True;break
                time.sleep(.01)
            if item['residual_group']:
                os.killpg(child.pid,signal.SIGKILL)
                raise RuntimeError((label,'residual process group force-reaped'))
            if timed_out:raise TimeoutError((label,timeout))
            if item['returncode']:raise RuntimeError((label,item['returncode']))
    try:
        execute('harness_regression',[sys.executable,'-B','r407/tests/test_operator_receipt.py'],root,10)
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'src/arena_autonomy',120)
        binaries=list((root/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'))
        assert len(binaries)==1;native={p.relative_to(root).as_posix():digest(p) for p in binaries}
        execute('preflight',[sys.executable,'-B','r407/preflight.py'],root,30)
        execute('ros_pty',[sys.executable,'-B','r407/run_once.py','--output',str(root/'results')],root,70)
    except Exception as exc:error=repr(exc)
    finally:
        source_changes=changed();prep_changes=preparation_changes()
        result=dict(execution_completed=error is None and not source_changes and not prep_changes,error=error,
            steps=steps,root=str(root),manifest_sha256=manifest_sha,source_changes=source_changes,
            preparation_changes=prep_changes,source_files=len(frozen),native_binaries=native,
            pythonpath=dict(isolated_first=isolated,inherited_after=inherited),domain=219,localhost_only=True,
            test_kind='actual ROS plus fresh PTY fake MCU only',physical_devices_opened=False,
            firmware_written=False,simulation_started=False,positive_drive_tested=False)
        (root/'execution.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
        shutil.copyfile(root/'execution.json',receipt/'execution.json')
        with zipfile.ZipFile(receipt/'results.zip','x',zipfile.ZIP_DEFLATED) as z:
            for path in [root/'execution.json',root/'preflight.json',*root.glob('*.log'),*(root/'results').rglob('*')]:
                if path.is_file():z.write(path,path.relative_to(root).as_posix())
        print(json.dumps(result))
    if error or source_changes or prep_changes:raise SystemExit(1)


if __name__=='__main__':main()
