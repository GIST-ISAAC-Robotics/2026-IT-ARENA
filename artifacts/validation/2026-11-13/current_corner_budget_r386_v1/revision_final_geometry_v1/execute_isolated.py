"""지정 담당자만 fresh WSL x86 root에서 저장 비교를 실행한다. 장치/ROS 없음."""
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
    parser=argparse.ArgumentParser();parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--sha256',required=True);parser.add_argument('--root',type=Path,required=True);args=parser.parse_args()
    archive=args.archive.resolve();root=args.root.resolve()
    assert sys.platform=='linux' and os.uname().machine=='x86_64','assigned WSL executor only'
    assert root.is_relative_to(Path.home()) and root.name.startswith('it-arena-r386-final-') and not root.exists()
    assert digest(archive)==args.sha256;root.mkdir()
    with zipfile.ZipFile(archive) as zipped:
        for name in zipped.namelist():assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
        zipped.extractall(root)
    work=root/'r386';manifest=json.loads((work/'source_manifest.json').read_bytes())
    def changed():return [n for n,h in manifest['files'].items() if not (root/n).is_file() or digest(root/n)!=h]
    assert not changed() and digest(Path(__file__))==digest(work/'execute_isolated.py')
    frozen=root/'frozen';geometry=root/'geometry';results=work/'results_native_v1';results.mkdir()
    def environment(source):
        return dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
            PYTHONPATH=os.pathsep.join(str(p) for p in (work,source/'src/arena_autonomy',source/'src/arena_vehicle_interface',source/'scripts')),
            R386_SOURCE_ROOT=str(source),R386_INPUT_ROOT=str(root/'inputs'))
    steps=[];failure=None;binary_hashes={}
    def execute(label,command,cwd,source,timeout):
        with (work/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=environment(source),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            step=dict(label=label,command=command,pid=child.pid,timeout_s=timeout,forced=False);steps.append(step)
            try:step['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                step['forced']=True;os.killpg(child.pid,signal.SIGTERM)
                try:step['returncode']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);step['returncode']=child.wait(timeout=5)
                raise
            if step['returncode']:raise RuntimeError((label,step['returncode']))
    try:
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],frozen/'src/arena_autonomy',frozen,120)
        binaries=list((frozen/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'));assert len(binaries)==1
        binary=binaries[0];target=geometry/binary.relative_to(frozen);shutil.copyfile(binary,target)
        binary_hashes={str(p.relative_to(root)):digest(p) for p in (binary,target)};assert len(set(binary_hashes.values()))==1
        execute('tests',[sys.executable,'-B','-m','pytest','-q',str(work/'test_candidate.py'),
            str(geometry/'tests/test_vehicle_geometry_contract.py')],geometry,geometry,120)
        for mode in ('frozen','legacy','final'):
            source=frozen if mode=='frozen' else geometry
            command=[sys.executable,'-B',str(work/'compare.py'),'--mode',mode,'--output',str(results/mode)]
            if mode!='frozen':command+=['--reference',str(results/('frozen' if mode=='legacy' else 'legacy'))]
            execute(mode,command,root,source,900)
        execute('aggregate',[sys.executable,'-B',str(work/'aggregate.py'),'--output',str(results)],root,geometry,120)
    except Exception as exc:failure=repr(exc)
    finally:
        changes=changed();receipt=dict(execution_completed=failure is None and not changes,error=failure,steps=steps,
            changed_sources=changes,source_count=len(manifest['files']),archive_sha256=args.sha256,native_binaries=binary_hashes,
            original_r386_archive_executed=False,devices_opened=False,simulations_started=False,default_changed=False)
        (work/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8')
        target=archive.with_name(archive.stem+'-results.zip');assert not target.exists()
        with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED) as zipped:
            for path in [*work.glob('*.log'),work/'execution.json',*results.rglob('*')]:
                if path.is_file():zipped.write(path,path.relative_to(work).as_posix())
        print(json.dumps(receipt))
    if failure or changes:raise SystemExit(1)


if __name__=='__main__':main()
