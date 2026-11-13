"""r336/r361을 유지하고 현재 검사 r375을 명시 선택한다."""
import hashlib
import importlib.util
import os
from pathlib import Path
import runpy
import sys

A = Path(__file__).resolve().parent
BASE = A.parent/'current_ros_bridge_r336_v1'
sys.path.insert(0, str(BASE))
spec = importlib.util.spec_from_file_location('r336_entry_base', BASE/'node_entry.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
sys.path.insert(0, str(A))
import low_solo_worker
assert Path(low_solo_worker.__file__).resolve() == A/'low_solo_worker.py', 'wrong worker module'


def install(profile):
    restore = base.install(profile)
    sys.modules['worker_candidate'].install = low_solo_worker.install
    from arena_autonomy.g1_g2_node import G1G2Node
    inner = G1G2Node.__init__
    def initialize(node):
        inner(node)
        source = A.parent/'released_current_r375_v1/candidate.py'
        node.follow_trace.emit('current_relative_body_policy', dict(policy='r375',
            sha256=hashlib.sha256(source.read_bytes()).hexdigest(), cap_mps=.10,
            full_current_check=True, default_changed=False, hardware_enabled=False))
        source = A.parent/'released_crawl_r361_v1/candidate.py'
        node.follow_trace.emit('released_plan_crawl_policy', dict(policy='r361',
            sha256=hashlib.sha256(source.read_bytes()).hexdigest(), stage='plan_only', cap_mps=.10,
            previous_caps_mps=[.35, .25, .15], full_current_region_check=True,
            raw_measured_speed_preserved=True, source_timestamps_unchanged=True,
            unknown_filled=False, masked_tail_allowance_m=.01,
            current_recheck_policy='r375_released_relative_body', default_changed=False, hardware_enabled=False,
            child_receipt_directory=os.environ.get('R377_POLICY_RECEIPT_DIR', str(A/'worker_policy_receipts')),
            receipt_scope='parent configuration only; actual child installation recorded separately'))
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
