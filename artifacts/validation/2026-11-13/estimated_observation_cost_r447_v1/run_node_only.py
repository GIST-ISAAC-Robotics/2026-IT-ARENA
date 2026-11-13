"""기존 Humble 환경을 source한 뒤 생성자 단계만 재개. 원 비용/실패 보존."""
from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,sys,time,zipfile
ROOT=Path('/home/q/it-arena-r447-a')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    manifest=json.loads((ROOT/'source_manifest.json').read_bytes())
    assert all(sha(ROOT/n)==h for n,h in manifest['files'].items())
    before={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'cost').rglob('*') if p.is_file()}
    out=ROOT/'node_check_v2';out.mkdir(exist_ok=False)
    for name in ('verify_nodes.py','runtime_profile.yaml','bindings.json'):
        shutil.copyfile(ROOT/'node_check'/name,out/name)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_DOMAIN_ID='147',ROS_LOCALHOST_ONLY='1')
    error=None;forced=False
    with (out/'execution.log').open('x') as log:
        child=subprocess.Popen([sys.executable,'-B',str(out/'verify_nodes.py')],cwd=ROOT,env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=child.wait(timeout=90)
        except subprocess.TimeoutExpired:
            forced=True;error='node constructor timeout';os.killpg(child.pid,signal.SIGINT)
            try:code=child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);code=child.wait(timeout=3)
    try:os.killpg(child.pid,0);residual=True
    except ProcessLookupError:residual=False
    if residual:os.killpg(child.pid,signal.SIGKILL)
    changed=[n for n,h in manifest['files'].items() if sha(ROOT/n)!=h]
    cost_changed=[n for n,h in before.items() if sha(ROOT/n)!=h]
    report=json.loads((out/'report.json').read_bytes()) if (out/'report.json').exists() else None
    receipt=dict(exitcode=code,pid=child.pid,residual_group=residual,forced=forced,error=error,
        changed_sources=changed,cost_changes=cost_changed,calculation_repeated=False,
        ros_environment='/opt/ros/humble/setup.bash',report_passed=None if report is None else report['passed'])
    (out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    archive=ROOT.with_name(ROOT.name+'-node-v2-results.zip')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in out.iterdir():
            if p.is_file():z.write(p,p.name)
    print(json.dumps(receipt));print(json.dumps(dict(archive=str(archive),sha256=sha(archive))))
    return 0 if code==0 and not residual and not changed and not cost_changed else 1
if __name__=='__main__':raise SystemExit(main())
