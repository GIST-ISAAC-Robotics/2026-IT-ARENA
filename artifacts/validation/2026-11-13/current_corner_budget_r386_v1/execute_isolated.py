"""주 에이전트가 배정한 WSL 담당자만 실행. 새 루트 빌드/회귀/저장 비교 각1회."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import zipfile


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--sha256',required=True)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();archive=args.archive.resolve();root=args.root.resolve()
    assert sys.platform=='linux' and os.uname().machine=='x86_64','assigned WSL x86 executor only'
    assert root.is_relative_to(Path.home()) and root.name.startswith('it-arena-r386-') and not root.exists()
    assert digest(archive)==args.sha256
    root.mkdir()
    with zipfile.ZipFile(archive) as zipped:
        for name in zipped.namelist():assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
        zipped.extractall(root)
    work=root/'artifacts/validation/2026-11-13/current_corner_budget_r386_v1'
    manifest=json.loads((work/'source_manifest.json').read_bytes())
    def changed():return [n for n,h in manifest['files'].items() if digest(root/n)!=h]
    assert not changed()
    assert digest(Path(__file__))==digest(work/'execute_isolated.py')
    env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(p) for p in (work,root/'src/arena_autonomy',root/'src/arena_vehicle_interface')),
        PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    steps=[];failure=None
    def execute(label,command,cwd,timeout):
        with (work/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            step=dict(label=label,command=command,pid=child.pid,forced=False,timeout_s=timeout);steps.append(step)
            try:step['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                step['forced']=True;os.killpg(child.pid,signal.SIGTERM)
                try:step['returncode']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);step['returncode']=child.wait(timeout=5)
                raise
            if step['returncode']:raise RuntimeError((label,step['returncode']))
    try:
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'src/arena_autonomy',120)
        execute('tests',[sys.executable,'-B','-m','pytest','-q',str(work/'test_candidate.py')],root,120)
        execute('saved_compare',[sys.executable,'-B',str(work/'compare.py'),'--backend','native_v1','--output',str(work/'results_native_v1')],root,900)
    except Exception as exc:failure=repr(exc)
    finally:
        changes=changed()
        receipt=dict(execution_completed=failure is None and not changes,error=failure,steps=steps,
            changed_sources=changes,archive_sha256=args.sha256,source_count=len(manifest['files']),
            devices_opened=False,simulations_started=False,default_changed=False)
        (work/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8')
        result=archive.with_name(archive.stem+'-results.zip')
        assert not result.exists()
        with zipfile.ZipFile(result,'x',zipfile.ZIP_DEFLATED) as zipped:
            for path in [*work.glob('*.log'),work/'execution.json',*work.glob('results_native_v1/*')]:
                if path.is_file():zipped.write(path,path.relative_to(work).as_posix())
        print(json.dumps(receipt))
    if failure or changes:raise SystemExit(1)


if __name__=='__main__':main()
