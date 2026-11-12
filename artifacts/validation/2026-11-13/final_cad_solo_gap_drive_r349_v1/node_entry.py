"""r336 발행/두 슬롯 연결을 유지하며 r346 현재 검사만 명시 선택."""
import hashlib
import importlib.util
from pathlib import Path
import runpy
import sys

A = Path(__file__).resolve().parent
BASE = A.parent/'current_ros_bridge_r336_v1'
sys.path.insert(0, str(BASE))
spec = importlib.util.spec_from_file_location('r336_entry_base', BASE/'node_entry.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
import low_solo_worker


def install(profile):
    restore = base.install(profile)
    sys.modules['worker_candidate'].install = low_solo_worker.install
    from arena_autonomy.g1_g2_node import G1G2Node
    inner = G1G2Node.__init__
    def initialize(node):
        inner(node)
        source = A.parent/'current_solo_gap_r346_v1/candidate.py'
        node.follow_trace.emit('current_low_solo_policy', dict(policy='r346',
            sha256=hashlib.sha256(source.read_bytes()).hexdigest(), cap_mps=.15,
            full_current_check=True, default_changed=False, hardware_enabled=False))
    G1G2Node.__init__ = initialize
    return restore


def main():
    profile, executable, *arguments = sys.argv[1:]
    if Path(executable).name != 'g1_g2_handoff_trial':
        raise ValueError('unexpected executable')
    install(profile)
    sys.argv = [executable, *arguments]
    runpy.run_path(executable, run_name='__main__')


if __name__ == '__main__':
    main()
