"""Windows 격리 사본만 생성/검증한다. 공통/WSL/Jetson/ROS 장치 실행 없음."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,ast,hashlib,json,os,shutil,signal,subprocess,sys,time

BASE=Path(__file__).resolve().parent;REPO=BASE.parents[3]
TESTS=['test_runtime_plan_trace.py','test_runtime_dual_slots.py','test_runtime_dual_contract.py',
       'test_runtime_node_backend.py','test_runtime_dual_spawn.py']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('label');parser.add_argument('--test',action='append',choices=TESTS);a=parser.parse_args()
    assert a.label.startswith('verify_windows_') and '/' not in a.label and '\\' not in a.label
    before=json.loads((BASE/'source_before.json').read_bytes())
    assert all(sha(REPO/n)==h for n,h in before.items()),'common changed since original stage copy'
    output=BASE/a.label;output.mkdir(exist_ok=False);work=output/'workspace';work.mkdir()
    inputs={}
    for package in ('arena_autonomy','arena_vehicle_interface'):
        for p in sorted((REPO/'src'/package/package).rglob('*.py')):
            name=p.relative_to(REPO).as_posix();dest=work/name;dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(p,dest);inputs[name]=dict(original_sha256=sha(p))
    (work/'tests').mkdir()
    for name in TESTS:
        p=REPO/'tests'/name
        if p.exists():shutil.copyfile(p,work/'tests'/name)
    for p in (BASE/'stage').rglob('*'):
        if p.is_file():
            dest=work/p.relative_to(BASE/'stage');dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
    for p in work.rglob('*.py'):ast.parse(p.read_text(encoding='utf8'))
    shutil.copyfile(BASE/'run_tests.py',work/'run_tests.py')
    manifest={p.relative_to(work).as_posix():sha(p) for p in work.rglob('*') if p.is_file()}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    command=[sys.executable,'-B',str(work/'run_tests.py'),*('tests/'+n for n in (a.test or TESTS))]
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',PYTHONIOENCODING='utf-8',
        PYTHONPATH=os.pathsep.join(str(p) for p in (work/'src/arena_autonomy',work/'src/arena_vehicle_interface',
            work/'tests',BASE/'test_dependencies')))
    began=time.monotonic();forced=False
    with (output/'execution.log').open('x',encoding='utf8') as log:
        child=subprocess.Popen(command,cwd=work,env=env,stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
        try:code=child.wait(timeout=120)
        except subprocess.TimeoutExpired:
            forced=True;child.send_signal(signal.CTRL_BREAK_EVENT)
            try:code=child.wait(timeout=10)
            except subprocess.TimeoutExpired:child.terminate();code=child.wait(timeout=5)
    changes=[n for n,h in manifest.items() if sha(work/n)!=h]
    common_changes=[n for n,h in before.items() if sha(REPO/n)!=h]
    report=json.loads((work/'test_receipt.json').read_bytes()) if (work/'test_receipt.json').is_file() else None
    receipt=dict(completed_at_utc=datetime.now(timezone.utc).isoformat(),command=command,pytest_pid=child.pid,
        exitcode=code,pytest_process_alive=child.poll() is None,forced=forced,wall_s=time.monotonic()-began,
        input_changes=changes,common_changes=common_changes,source_count=len(manifest),
        test_receipt=report,wsl_used=False,jetson_used=False,ros_used=False)
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps({k:v for k,v in receipt.items() if k!='test_receipt'}))
    if report:print(json.dumps(dict(outcome_counts={k:sum(r['outcome']==k for r in report['outcomes'])
        for k in ('passed','failed','skipped')},owned_remaining=report['owned_remaining'],external_imports=report['external_imports'])))
    return code or int(bool(changes or common_changes or forced or report is None or report['owned_remaining']))
if __name__=='__main__':raise SystemExit(main())
