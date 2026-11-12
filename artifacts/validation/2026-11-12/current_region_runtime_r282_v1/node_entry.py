"""명시 prefix의 단일 제어 노드에만 병렬 후보를 연결한다. 실물 환경은 거부한다."""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import runpy
import sys
from delivery import Delivery, Worker
from worker_candidate import install


def main():
    profile,executable,*arguments=sys.argv[1:]
    if Path(executable).name!='g1_g2_handoff_trial':raise ValueError('unexpected executable')
    record=json.loads(Path(profile).read_bytes())
    assert record['schema']=='r274_parallel_sim_clock_v1'
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.local_pursuit import LocalPursuit
    from arena_autonomy.completion_notice import CompletionNotice
    from arena_autonomy.follow_trace import snapshot_call
    original=G1G2Node.__init__

    def initialize(node):
        original(node)
        assert node.get_parameter('use_sim_time').value and not node.hardware_shadow
        assert node.recheck_in_worker and node.decision_delivery.retain_expired_reference
        assert not node.decision_notice.close()['alive']
        old=node.decision_delivery.worker.close()
        assert not old['forced'] and old['exitcode']==0
        install()
        deadline=[None]
        def wake():
            timer.cancel();deadline[0]=None
            node.decision_delivery.collect();node.control()
        timer=node.create_timer(3600.,wake);timer.cancel()
        def arm(due):
            if deadline[0] is not None and deadline[0]<=due:return
            deadline[0]=due
            timer.timer_period_ns=max(1000,math.ceil(max(1e-6,due-node.now_s())*1e9))
            timer.reset()
        header=dict(schema=5,controller_mode='g1_g2_handoff_trial',numeric_execution='native_v1',
                    motion_config=asdict(node.motion.history.config),follow_config=asdict(node.follow.config),
                    execution_config=asdict(node.follow.execution))
        worker=Worker(header,clock=node.now_s,delays=(record['plan_delay_s'],record['recheck_delay_s']),wake=arm)
        node.decision_delivery=Delivery(worker,node.now_s,node.follow_trace.emit)
        node.decision_notice=CompletionNotice(worker.connection,node.decision_guard.trigger)
        node.follow_trace.emit('parallel_runtime_policy',dict(profile_sha256=hashlib.sha256(Path(profile).read_bytes()).hexdigest(),
            plan_delay_s=record['plan_delay_s'],recheck_delay_s=record['recheck_delay_s'],
            submission='latest_unused_native_v1',retained_fallback=False,geometry_budget_s=.5,
            current_command_budget_s=.2,current_scan_budget_s=.3,default_changed=False,
            original_reference_preserved=True,hardware_enabled=False))
        node.follow_trace.emit('current_region_policy',dict(candidate='r281_current_low_speed_gap',unknown_filled=False,old_occupancy_permission_reused=False,default_changed=False))
        assert node.follow.config.standing_wall_reacquire and node.follow.config.junction_continuity
        node.follow_trace.emit('opposite_reacquire_policy',dict(candidate='r278_standing_observed_opposite_wall',
            measured_speed_limit_mps=.10,preferred_path_must_be_absent=True,
            unknown_filled=False,default_changed=False))

    def control(node):
        delivery=node.decision_delivery
        if delivery is None:return LocalPursuit.control(node)
        notice=node.decision_notice
        if notice is not None and notice.fault:delivery.worker.fault='completion_notice: '+notice.fault
        node.async_inputs_ready=False
        node.decision_command_stamp_s=None
        node.decision_output_dt_s=max(0.,min(.1,node.now_s()-node.last_request_time_s))
        result=LocalPursuit.control(node)
        if node.command_was_published:
            node.last_request_time_s=node.now_s()
            delivery.last_application=node.last_request_time_s
        # 현재 검사와 계획은 서로 다른 슬롯이다. 현재 검사 진행 중에도 신규 계획은 가능하다.
        w=delivery.worker
        if node.async_inputs_ready and not w.fault and w.plan.inflight is None and w.saved_plan is None:
            reference=node.motion.last_meta['reference_stamp_s']
            speed=node.motion.history.wheels.values[-1] if node.motion.history.wheels.values else 0.
            snapshot=snapshot_call(node.scan,node.motion.history,reference,node.motion.counts['resets'],
                speed,node.side,node.last_steering,.1,node.motion.verified,node.rear_blind_half_angle,
                timing_policy=getattr(node.motion,'timing_policy','verified_only'))
            delivery.offer(snapshot)
        return result

    G1G2Node.__init__=initialize
    G1G2Node.control=control
    sys.argv=[executable,*arguments]
    runpy.run_path(executable,run_name='__main__')


if __name__=='__main__':main()
