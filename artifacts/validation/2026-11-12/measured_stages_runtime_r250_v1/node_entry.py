"""평가 launch의 G1/G2 한 노드에만 두 단계 최소 대기 시간을 넣는다."""
from pathlib import Path
import hashlib,json,runpy,sys
from staged_delay import StagedDelay


def main():
    profile,executable,*arguments=sys.argv[1:]
    if Path(executable).name!='g1_g2_handoff_trial':raise ValueError('unexpected executable')
    record=json.loads(Path(profile).read_bytes())
    assert record['schema']=='r250_stage_p95_minimum_age_v1'
    from arena_autonomy.g1_g2_node import G1G2Node
    original=G1G2Node.__init__
    def initialize(node):
        original(node)
        assert node.get_parameter('use_sim_time').value and not node.hardware_shadow
        assert node.recheck_in_worker and node.decision_delivery.retain_expired_reference
        node.decision_delivery.worker=StagedDelay(node.decision_delivery.worker,node.now_s,
            record['plan_delay_s'],record['recheck_delay_s'])
        node.follow_trace.emit('delivery_delay_profile',dict(profile_sha256=hashlib.sha256(Path(profile).read_bytes()).hexdigest(),
            plan_delay_s=record['plan_delay_s'],recheck_delay_s=record['recheck_delay_s'],
            scope=record['scope'],default_changed=False,original_reference_preserved=True))
    G1G2Node.__init__=initialize
    sys.argv=[executable,*arguments];runpy.run_path(executable,run_name='__main__')


if __name__=='__main__':main()
