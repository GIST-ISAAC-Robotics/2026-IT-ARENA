"""평가 전용 고정 지연. 원 단계 비용을 유지하며 지연 만료를 비동기 알림처럼 전달."""
from pathlib import Path
import hashlib,json,math,runpy,sys
from staged_delay import EventStagedDelay


def main():
    profile,executable,*arguments=sys.argv[1:]
    if Path(executable).name!='g1_g2_handoff_trial':raise ValueError('unexpected executable')
    record=json.loads(Path(profile).read_bytes())
    assert record['schema']=='r252_stage_p95_event_age_v1'
    from arena_autonomy.g1_g2_node import G1G2Node
    original=G1G2Node.__init__
    def initialize(node):
        original(node)
        assert node.get_parameter('use_sim_time').value and not node.hardware_shadow
        assert node.recheck_in_worker and node.decision_delivery.retain_expired_reference
        def wake():
            timer.cancel();proxy.notified()
            node.decision_delivery.collect();node.control()
        timer=node.create_timer(3600.,wake);timer.cancel()
        def arm(seconds):
            timer.timer_period_ns=max(1000,math.ceil(seconds*1e9));timer.reset()
        proxy=EventStagedDelay(node.decision_delivery.worker,node.now_s,
            record['plan_delay_s'],record['recheck_delay_s'],arm,timer.cancel)
        node.decision_delivery.worker=proxy
        node.follow_trace.emit('delivery_delay_profile',dict(profile_sha256=hashlib.sha256(Path(profile).read_bytes()).hexdigest(),
            plan_delay_s=record['plan_delay_s'],recheck_delay_s=record['recheck_delay_s'],
            scope=record['scope'],wakeup='sim_clock_one_shot',default_changed=False,original_reference_preserved=True))
    G1G2Node.__init__=initialize
    sys.argv=[executable,*arguments];runpy.run_path(executable,run_name='__main__')


if __name__=='__main__':main()
