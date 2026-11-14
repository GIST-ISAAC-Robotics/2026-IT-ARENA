"""승인 전 공통 파일을 변경하지 않는 controller 전용 예산 이관안."""
import difflib,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];DEST=A/'common_transfer'
DEST.mkdir(exist_ok=False)
names=['src/arena_autonomy/arena_autonomy/lidar_motion_ros.py','src/arena_autonomy/arena_autonomy/g1_g2_node.py']
diff=[];manifest={}
for name in names:
    raw=(ROOT/name).read_bytes();old=raw.decode('utf8').replace('\r\n','\n');new=old
    if name.endswith('lidar_motion_ros.py'):
        a='    def drain_pending(self):';b='    def drain_pending(self, *, budget_s=.002):'
        assert new.count(a)==1;new=new.replace(a,b,1)
        a='        self.last_drain = drain_ready_motion(self._ready_motion_subscriptions)'
        b='        self.last_drain = drain_ready_motion(self._ready_motion_subscriptions, budget_s=budget_s)'
        assert new.count(a)==1;new=new.replace(a,b,1)
    else:
        a='        self.motion.drain_pending()'
        b='        self.motion.drain_pending(budget_s=.006)'
        assert new.count(a)==1;new=new.replace(a,b,1)
    compile(new,name,'exec')
    path=DEST/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(new.encode('utf8'))
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    manifest[name]=dict(base_sha256=hashlib.sha256(raw).hexdigest(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
(DEST/'controller_budget.patch').write_bytes(''.join(diff).encode('utf8'))
(DEST/'manifest.json').write_bytes((json.dumps(manifest,indent=2)+'\n').encode('utf8'))
print(json.dumps(dict(files=2,patch_sha256=hashlib.sha256((DEST/'controller_budget.patch').read_bytes()).hexdigest(),manifest=manifest)))
