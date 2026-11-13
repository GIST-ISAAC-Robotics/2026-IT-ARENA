"""ROS 클래스 import/명시 프로필·host 정책 검사. Node/serial 생성 없음."""
import hashlib
import json
from pathlib import Path
import sys

from registration import read
from arena_autonomy.g1_g2_node import G1G2Node
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.race_ops_node import RaceOps
from run_runtime_actuation_host import load_policy


def main():
    here=Path(__file__).resolve().parent;root=here.parent
    profile,bindings,report=read(here)
    policy,declared=load_policy(here/'prepared/clock_policy_pty.json')
    assert policy.same_host.source_ttl_ns==policy.receiver_lease.source_ttl_ns==100_000_000
    imports={}
    for name,module in sorted(sys.modules.items()):
        if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module,'__file__',None):
            path=Path(module.__file__).resolve();assert path.is_relative_to(root),(name,path)
            imports[name]=dict(path=path.relative_to(root).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    result=dict(passed=True,registration=report,clock_policy=declared,imports=imports,
        node_instantiated=False,serial_opened=False,physical_calibration_verified=False)
    (root/'preflight.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='imports'}))


if __name__=='__main__':main()
