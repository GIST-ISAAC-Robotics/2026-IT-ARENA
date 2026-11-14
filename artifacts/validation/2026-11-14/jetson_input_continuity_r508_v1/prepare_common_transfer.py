"""공통 파일은 건드리지 않고 검토용 4파일 diff/해시를 만든다."""
import difflib,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent
ROOT=A.parents[3]
DEST=A/'common_transfer'
DEST.mkdir(exist_ok=True)
paths=['src/arena_autonomy/arena_autonomy/lidar_motion_ros.py',
       'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
       'src/arena_autonomy/arena_autonomy/lidar_safety.py',
       'src/arena_autonomy/arena_autonomy/race_runtime_policy/current_route_priority.py']
diff=[];manifest={}
for name in paths:
    path=ROOT/name;raw=path.read_bytes();old=raw.decode('utf8').replace('\r\n','\n');new=old
    if name.endswith('lidar_motion_ros.py'):
        new=new.replace("        node.create_subscription(JointState, self.feedback_topic, wrap('wheels', callback), 100)",
            "        self._ready_motion_subscriptions = [node.create_subscription(\n"
            "            JointState, self.feedback_topic, wrap('wheels', callback), 100)]",1)
        new=new.replace("        node.create_subscription(Imu, imu_topic, wrap('imu', self.on_imu), imu_qos)",
            "        self._ready_motion_subscriptions.append(node.create_subscription(\n"
            "            Imu, imu_topic, wrap('imu', self.on_imu), imu_qos))",1)
        insert='''    def drain_pending(self):
        """현재 control callback 안에서 이미 준비된 표본만 기존 callback으로 소비한다.

        control과 운동 subscriptions는 같은 기본 mutually-exclusive callback group을
        사용한다. 호출 뒤 제어 함수가 now를 읽는다. 동시 직접 호출은 지원하지 않는다.
        상한은 메시지 사이에서 검사하며 남은 표본과 원 stamp/만료 기준은 보존한다.
        """
        self.last_drain = drain_ready_motion(self._ready_motion_subscriptions)
        return self.last_drain

'''
        new=new.replace('    def clock(self):',insert+'    def clock(self):',1)
        helper=(A/'motion_ingress_candidate.py').read_text(encoding='utf8')
        helper=helper[helper.index('def drain_ready('):].replace('def drain_ready(','def drain_ready_motion(',1)
        new=new.replace('class MotionInput:',helper+'\n\nclass MotionInput:',1)
    elif name.endswith('g1_g2_node.py'):
        new=new.replace('    def control(self):\n',
            '    def control(self):\n        self.motion.drain_pending()\n',1)
    elif name.endswith('lidar_safety.py'):
        new=new.replace('    def control(self):\n',
            '    def control(self):\n        self.motion.drain_pending()\n',1)
    else:
        new=(A/'candidate_shm16_drain_proof/current_route_priority.py').read_text(encoding='utf8')
    assert old!=new,name
    compile(new,name,'exec')
    target=DEST/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(new.encode('utf8'))
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    manifest[name]=dict(base_sha256=hashlib.sha256(raw).hexdigest(),candidate_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
(DEST/'common_transfer.patch').write_bytes(''.join(diff).encode('utf8'))
(DEST/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(files=4,patch_sha256=hashlib.sha256((DEST/'common_transfer.patch').read_bytes()).hexdigest(),manifest=manifest)))
