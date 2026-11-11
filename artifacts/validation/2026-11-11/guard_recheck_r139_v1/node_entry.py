"""ros2 launch prefix로만 선택하는 시험 노드. 운영 진입점은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import runpy
import sys

from delay_worker import DelayedWorker
from fresh_delivery import FreshStartDelivery


def read_profile(path):
    record=json.loads(Path(path).read_bytes())
    if record.get('schema')!='r106_measured_delivery_v1':
        raise ValueError('registered delivery profile required')
    return record,[(row['offset_s'],row['delay_s']) for row in record['samples']]


def main():
    profile, executable, *arguments=sys.argv[1:]
    if Path(executable).name!='g1_g2_handoff_trial':
        raise ValueError('delay prefix applies only to G1/G2 trial executable')
    record,timeline=read_profile(profile)
    from candidate import install, worker_entry
    install()
    import arena_autonomy.decision_process as decision_process
    decision_process._worker=worker_entry
    from arena_autonomy.g1_g2_node import G1G2Node
    original=G1G2Node.__init__
    def initialize(node):
        original(node)
        if (node.decision_delivery is None or
                node.get_parameter('use_sim_time').value is not True):
            raise ValueError('delay trial requires simulation process execution')
        node.decision_delivery=FreshStartDelivery(node.decision_delivery.worker,align_first_source=True)
        node.decision_delivery.worker=DelayedWorker(
            node.decision_delivery.worker,node.now_s,timeline)
        node.follow_trace.emit('delivery_delay_profile',dict(
            profile_sha256=hashlib.sha256(Path(profile).read_bytes()).hexdigest(),
            samples=len(timeline),scope=record['scope'],completion_window_s=.025,
            submission_policy='fresh_completed_scan_r114',current_follow_rebase='follow_baseline_parallel_once_v2',
            origin='first_submitted_reference',default_changed=False,body_nonreturn_candidate='guard_low_speed_body_nonreturn_r136',guard_recheck='bounded_body_full_guard_recheck_r139'))
    G1G2Node.__init__=initialize
    from trial_command import command
    G1G2Node.command=command
    sys.argv=[executable,*arguments]
    runpy.run_path(executable,run_name='__main__')


if __name__=='__main__':main()
