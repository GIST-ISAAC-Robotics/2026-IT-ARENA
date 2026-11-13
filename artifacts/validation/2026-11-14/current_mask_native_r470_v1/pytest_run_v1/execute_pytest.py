"""기존 r470 native/source는 읽기만 하고 신규 pytest10건을 한 번 실행한다."""
import hashlib,json,os
from pathlib import Path
import signal,subprocess,sys,traceback,xml.etree.ElementTree as ET,zipfile

ROOT=Path(__file__).resolve().parent
SOURCE=Path('/home/q/it-arena-r470-a')
sys.path.insert(0,str(SOURCE))
from environment_reference import other_work
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert ROOT==Path('/home/q/it-arena-r470-tests-a') and os.uname().machine=='aarch64'
    assert not (ROOT/'receipt.json').exists() and not (ROOT/'pytest.log').exists()
    manifest=SOURCE/'source_manifest.json'
    assert sha(manifest)=='fa417a2a37c499556771b8299a395bf5065e8c05de3672a8d85ab691520046e2'
    source_files=json.loads(manifest.read_bytes())['files']
    native=SOURCE/'stage/src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so'
    native_hash='5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff'
    test=ROOT/'test_current_mask_native.py'
    test_hash='ecce49902cdd3dcad22e46578d79c0a6fcd0bde6898f3d872530cb805aa7e341'
    def changed():return [p for p,h in source_files.items() if sha(SOURCE/p)!=h]
    assert not changed() and sha(native)==native_hash and sha(test)==test_hash
    interference=other_work(ROOT);assert not any(interference.values()),interference
    environment=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        PYTHONPATH=os.pathsep.join([str(SOURCE/'stage/src/arena_autonomy'),str(SOURCE/'stage/src/arena_vehicle_interface')]))
    command=[sys.executable,'-B',str(ROOT/'run_pytest.py')]
    row=dict(command=command,timeout_s=60,forced=False)
    error=None;tests=[]
    try:
        with (ROOT/'pytest.log').open('x') as log:
            child=subprocess.Popen(command,cwd=ROOT,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            row['pid']=child.pid
            try:row['returncode']=child.wait(timeout=60)
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
        tree=ET.parse(ROOT/'pytest.xml')
        tests=[dict(name=n.attrib['name'],failed=bool(n.findall('failure')+n.findall('error')),
            skipped=bool(n.findall('skipped'))) for n in tree.findall('.//testcase')]
        assert len(tests)==10 and not any(t['failed'] or t['skipped'] for t in tests),tests
    except BaseException as exc:
        error=repr(exc);(ROOT/'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(completed=error is None,error=error,step=row,tests=tests,
            test_count=len(tests),test_sha256=sha(test),native_sha256=sha(native),
            expected_native_sha256=native_hash,source_changes=changed(),source_count=len(source_files),
            remaining=other_work(ROOT),new_build=False,packages_installed=False,
            source_changes_allowed=False,performance_matrix_executed=False,sensors_opened=False)
        assert sha(test)==test_hash
        (ROOT/'receipt.json').write_text(json.dumps(receipt,indent=2))
        archive=ROOT.with_name(ROOT.name+'-results.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for name in ('receipt.json','pytest.log','pytest.xml','imports.json','traceback.txt'):
                path=ROOT/name
                if path.is_file():z.write(path,name)
        print(json.dumps(dict(completed=receipt['completed'],error=error,step=row,test_count=len(tests),
            source_changes=receipt['source_changes'],remaining=receipt['remaining'],
            archive=str(archive),archive_sha256=sha(archive))))
    return 0 if error is None and not receipt['source_changes'] and sha(native)==native_hash and not any(receipt['remaining'].values()) else 1

if __name__=='__main__':raise SystemExit(main())
