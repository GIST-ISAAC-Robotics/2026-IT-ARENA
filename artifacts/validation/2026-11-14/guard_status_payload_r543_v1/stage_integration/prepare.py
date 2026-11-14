"""공통 lidar_safety를 읽어 비제어 status 연결만 stage 사본에 넣는다."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
source=ROOT/'src/arena_autonomy/arena_autonomy/lidar_safety.py'
raw=source.read_bytes();before=raw.decode('utf-8');after=before
needle='        self.status = self.create_publisher(String, "/safety/status", 10)'
assert after.count(needle)==1
after=after.replace(needle,needle+'''
        from arena_autonomy.guard_status_payload import make_encoder
        self.status_encoder = make_encoder(
            self.declare_parameter('safety_status_detail', 'full').value,
            self.declare_parameter('safety_status_evidence_path', '').value,
            self.get_logger().warning)''')
needle='self.status.publish(String(data=json.dumps({'
assert after.count(needle)==1
after=after.replace(needle,"self.status.publish(String(data=(getattr(self, 'status_encoder', None) or json.dumps)({")
needle='\n\ndef main(args=None):';assert after.count(needle)==1
after=after.replace(needle,'''
    def destroy_node(self):
        from arena_autonomy.guard_status_payload import close_encoder
        close_encoder(getattr(self, 'status_encoder', None))
        return super().destroy_node()


def main(args=None):''')
def control(text):
    cls=next(n for n in ast.parse(text).body if isinstance(n,ast.ClassDef) and n.name=='LidarSafety')
    return next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control')
original,stage=control(before),control(after)
assert ast.dump(ast.Module(body=original.body[:-1],type_ignores=[]))==ast.dump(ast.Module(body=stage.body[:-1],type_ignores=[]))
compile(after,'lidar_safety.pystage','exec')
(HERE/'lidar_safety.before.py').write_bytes(raw)
(HERE/'lidar_safety.pystage').write_text(after,encoding='utf-8')
(HERE/'integration.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
    fromfile='a/src/arena_autonomy/arena_autonomy/lidar_safety.py',tofile='b/src/arena_autonomy/arena_autonomy/lidar_safety.py')),encoding='utf-8')
manifest=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(raw).hexdigest(),
    stage_sha256=hashlib.sha256((HERE/'lidar_safety.pystage').read_bytes()).hexdigest(),
    helper_sha256=hashlib.sha256((HERE/'guard_status_payload.py').read_bytes()).hexdigest(),
    control_ast_before_status_publish_identical=True,production_modified=False)
(HERE/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(manifest,indent=2))
