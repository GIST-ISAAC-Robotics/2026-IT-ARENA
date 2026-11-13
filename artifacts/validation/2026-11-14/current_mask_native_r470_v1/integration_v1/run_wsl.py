"""할당된 WSL에서 기존 native 빌드·관련 회귀만 수행하고 소유 PID를 회수한다."""
import hashlib,json,os,signal,subprocess,sys,traceback
from pathlib import Path
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent
ROOT=A.parents[4]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def other_work():
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            exe=(p/'exe').resolve().name
            if exe in ('bash','sh','dash','init','wslhost','sshd','sshd-session'):continue
            cwd=(p/'cwd').resolve();argv=(p/'cmdline').read_bytes().split(b'\0')
            command=' '.join(a.decode(errors='replace') for a in argv)
            if cwd.is_relative_to(ROOT) or str(ROOT) in command or exe in ('gzserver','gzclient','gz','g++','cc1plus','colcon'):
                found.append(dict(pid=int(p.name),exe=exe,cwd=str(cwd),command=command))
        except (OSError,PermissionError,ProcessLookupError):pass
    return found

def main():
    assert sys.platform=='linux' and os.uname().machine=='x86_64'
    interference=other_work()
    if sys.argv[1]=='preflight':
        report=dict(machine=os.uname().machine,root=str(ROOT),interference=interference,resource_available=not interference)
        (A/'wsl_preflight.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
        return 0 if not interference else 1
    assert sys.argv[1]=='execute' and not interference,interference
    assert not (A/'wsl_receipt.json').exists()
    source=json.loads((A/'source_receipt.json').read_bytes())
    def changed():return [r['path'] for r in source['files'] if sha(ROOT/r['path'])!=r['after']]
    assert not changed()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        PYTHONPATH=os.pathsep.join([str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]))
    steps=[];error=None;tests=[]
    def run(label,command,cwd,timeout):
        with (A/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            row=dict(label=label,command=command,pid=child.pid,timeout_s=timeout,forced=False);steps.append(row)
            try:row['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                row['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:row['returncode']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);row['returncode']=child.wait(timeout=5)
            try:os.killpg(child.pid,0)
            except ProcessLookupError:row['residual_group']=False
            else:
                row['residual_group']=True;os.killpg(child.pid,signal.SIGKILL)
            assert row['returncode']==0 and not row['forced'] and not row['residual_group'],row
    try:
        run('old_binary',[sys.executable,'-B',str(A/'probe_native.py'),'old'],ROOT,30)
        run('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],ROOT/'src/arena_autonomy',180)
        run('new_binary',[sys.executable,'-B',str(A/'probe_native.py'),'new'],ROOT,30)
        run('pytest',[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',
            '--junitxml='+str(A/'pytest.xml'),'tests/test_current_mask_native.py',
            'tests/test_current_structure_shadow.py','tests/test_numeric_kernels.py'],ROOT,180)
        tests=[dict(name=n.attrib['name'],failed=bool(n.findall('failure')+n.findall('error')),
            skipped=bool(n.findall('skipped'))) for n in ET.parse(A/'pytest.xml').findall('.//testcase')]
        assert len(tests)>=29 and not any(t['failed'] or t['skipped'] for t in tests),tests
        assert len([t for t in tests if t['name'].startswith(('test_python_selection_keeps','test_explicit_native_rejects_old',
            'test_inclusive_distance_boundary','test_empty_query_or_return','test_extension_rejects_invalid_buffers_before_read'))])==10
    except BaseException as exc:
        error=repr(exc);(A/'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(completed=error is None,error=error,steps=steps,tests=tests,test_count=len(tests),
            source_changes_during_execution=changed(),remaining=other_work(),
            native_rebuilt=any(s['label']=='native_build' and s.get('returncode')==0 for s in steps),jetson_used=False,performance_matrix_executed=False,
            source_scope=[r['path'] for r in source['files']],root_docs_changed=False,commit_or_push=False)
        (A/'wsl_receipt.json').write_text(json.dumps(receipt,indent=2))
        print(json.dumps({k:v for k,v in receipt.items() if k not in ('tests','source_scope')}))
    return 0 if error is None and not receipt['source_changes_during_execution'] and not receipt['remaining'] else 1

if __name__=='__main__':raise SystemExit(main())
