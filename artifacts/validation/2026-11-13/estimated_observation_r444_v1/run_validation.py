"""r444 격리 unit/native 비교를 회수한다. ROS 노드/장치 실행은 없다."""
from pathlib import Path
import hashlib,json,subprocess,sys,time

ROOT=Path(__file__).resolve().parent
manifest=json.loads((ROOT/'manifest.json').read_bytes())
def changed():return [p for p,want in manifest.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=want]
assert not changed()
steps=[]
for label,cmd in [('units',[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',
        'tests/test_estimated_observation_scope.py','tests/test_current_corner_runtime.py','tests/test_estimated_motion_time.py',
        '--junitxml=units.xml']),('saved',[sys.executable,'-B','verify_saved.py'])]:
    started=time.monotonic()
    with (ROOT/(label+'.log')).open('xb') as log:
        child=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT);forced=False
        try:code=child.wait(timeout=180)
        except subprocess.TimeoutExpired:forced=True;child.kill();code=child.wait()
    steps.append(dict(label=label,command=cmd,pid=child.pid,exitcode=code,forced=forced,reaped=True,elapsed_s=time.monotonic()-started))
    if code:break
report=dict(passed=len(steps)==2 and all(s['exitcode']==0 and not s['forced'] for s in steps) and not changed(),
    steps=steps,source_changes=changed(),ros_nodes_started=False,devices_opened=False)
(ROOT/'execution.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report));raise SystemExit(0 if report['passed'] else 1)
