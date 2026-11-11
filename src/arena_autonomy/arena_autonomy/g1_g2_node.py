"""G1/G2 전환을 별도 실행 파일에서만 선택한다. 기존 launch의 기본값은 유지한다."""
from pathlib import Path
from dataclasses import asdict, replace
import math

from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig, CORRIDOR_POLICIES
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig, PREPARATION_POLICIES, PERSISTENCE_WINDOWS
from arena_autonomy.follow_trace import FollowTrace, freeze
from arena_autonomy.g2_entry_node import status_detail as g2_status
from arena_vehicle_interface.node_lifecycle import run_node


def status_detail(detail):
    result = {key:detail[key] for key in ('mode','handoff_state','mode_before','behavior','reason',
        'simulation_trial_only','automatic_race_mode_selection','output_requires_independent_guard','input_error')
        if key in detail}
    if isinstance(detail.get('normal'),dict):
        result['normal'] = {key:detail['normal'][key] for key in
            ('reason','behavior','selector_mode','selector_choice','selector_reason') if key in detail['normal']}
    departure = ((detail.get('normal') or {}).get('parallel_hold_trial') or {}).get('side_departure')
    if isinstance(departure, dict):
        result['normal']['side_departure'] = departure
    lane = (detail.get('normal') or {}).get('current_lane_preference')
    if isinstance(lane, dict):
        result['normal']['current_lane_preference'] = lane
    if isinstance(detail.get('bypass'),dict):
        result['bypass'] = g2_status(detail['bypass'])
    if isinstance(detail.get('persistence'),dict):
        result['persistence'] = detail['persistence']
    moving = detail.get('moving_pass')
    if isinstance(moving,dict):
        result['moving_pass'] = {key:moving[key] for key in
            ('ready','reason','completed','phase','target','admission') if key in moving}
        if isinstance(moving.get('entry'),dict):
            result['moving_pass']['entry_reason'] = moving['entry'].get('reason')
    if isinstance(detail.get('preparation'),dict):
        result['preparation'] = {key:detail['preparation'][key] for key in
            ('policy','applied','reason') if key in detail['preparation']}
    if isinstance(detail.get('follow_entry_space'),dict):
        result['follow_entry_space'] = {key:detail['follow_entry_space'][key] for key in
            ('applied','reason','rear_axle_to_patch_m','reserve_m','cap_mps',
             'hold_age_s','maximum_age_s','current_front_evidence','bypass_permission') if key in detail['follow_entry_space']}
    return freeze(result)


class G1G2Node(LocalPursuit):
    def __init__(self):
        self.decision_delivery = None
        super().__init__()
        execution = self.declare_parameter('g1_decision_execution', 'inline').value
        if execution not in ('inline', 'process_candidate'):
            raise ValueError('registered decision execution required')
        if execution == 'process_candidate':
            if self.get_parameter('use_sim_time').value is not True or self.settings['control_rate_hz'] != 50.:
                raise ValueError('process candidate requires simulation and a 50Hz receiving parent')
            from arena_autonomy.decision_process import DecisionProcess
            from arena_autonomy.decision_delivery import DecisionDelivery
            header = dict(schema=5, controller_mode='g1_g2_handoff_trial',
                motion_config=asdict(self.motion.history.config), follow_config=asdict(self.follow.config),
                execution_config=asdict(self.follow.execution))
            self.decision_delivery = DecisionDelivery(DecisionProcess(header), align_first_source=True)
            self.last_request_time_s = -math.inf
            self.follow_trace.emit('reset', dict(reason='process_candidate_start',
                execution=execution, decision_dt_s=.1, receive_rate_hz=50., align_first_source=True))

    def reset_follow(self, reason):
        if self.decision_delivery is not None:
            self.decision_delivery.invalidate()
        if getattr(getattr(self, 'follow_trace', None), 'closed', False):
            self.follow.reset()
            return
        super().reset_follow(reason)

    def stop(self):
        if self.decision_delivery is not None:
            self.reset_follow('explicit_stop')
            # WallFollow.stop의 실제 발행값은 0조향이다. 다음 입력도 그 값을 사용한다.
            self.last_steering = 0.
            self.last_request_time_s = self.now_s()
            self.decision_delivery.last_application = self.last_request_time_s
        super().stop()

    def control(self):
        delivery = self.decision_delivery
        if delivery is None:
            return super().control()
        self.async_inputs_ready = False
        self.decision_command_stamp_s = None
        self.decision_output_dt_s = max(0., min(.1, self.now_s()-self.last_request_time_s))
        result = super().control()
        if self.command_was_published:
            self.last_request_time_s = self.now_s()
            delivery.last_application = self.last_request_time_s
        # 발행한 이전 요청각을 포함해 새 입력을 만든다. 진행 중에는 대기 입력을 쌓지 않는다.
        if (self.async_inputs_ready and not delivery.worker.fault and delivery.active is None
                and delivery.worker.inflight is None and delivery.worker.pending is None):
            from arena_autonomy.follow_trace import snapshot_call
            first = self.scan.header.stamp.sec + self.scan.header.stamp.nanosec*1e-9
            if hasattr(self.scan, 'acquired_at_s'):
                first = self.scan.acquired_at_s[0]
            identity = (getattr(self.scan, 'source_epoch', 0), first)
            reference = self.motion.last_meta['reference_stamp_s']
            if identity != delivery.last_source and reference-delivery.last_submission >= .1-1e-9:
                speed = self.motion.history.wheels.values[-1] if self.motion.history.wheels.values else 0.
                snapshot = snapshot_call(self.scan, self.motion.history, reference,
                    self.motion.counts['resets'], speed, self.side, self.last_steering, .1,
                    self.motion.verified, self.rear_blind_half_angle)
                delivery.offer(snapshot)
        return result

    def destroy_node(self):
        if self.decision_delivery is not None:
            self.stop()
            cleanup = self.decision_delivery.worker.close()
            if cleanup and (cleanup['forced'] or cleanup['exitcode'] != 0):
                self.get_logger().warning('Decision process cleanup: '+str(cleanup))
        return super().destroy_node()

    def create_follow_trace(self, trace_path):
        enabled = self.declare_parameter('g1_g2_simulation_trial',False).value
        if enabled is not True or self.get_parameter('use_sim_time').value is not True:
            raise ValueError('G1/G2 handoff requires explicit simulation-only activation')
        # 기본은 기존 즉시 준비 감속이다. 새 관측 근거 조건은 이름으로 명시할 때만 쓴다.
        policy = self.declare_parameter('g1_g2_preparation_policy','front_patch_v0').value
        if type(policy) is not str or policy not in PREPARATION_POLICIES:
            raise ValueError('registered G1/G2 preparation policy required')
        window = self.declare_parameter('g1_g2_persistence_window','compare_then_trim_v0').value
        if type(window) is not str or window not in PERSISTENCE_WINDOWS:
            raise ValueError('registered G1/G2 persistence window required')
        entry_policy = self.declare_parameter('g1_g2_entry_policy','registered_v0').value
        corridor = self.declare_parameter('g1_g2_corridor_policy','off').value
        if type(entry_policy) is not str or type(corridor) is not str or corridor not in CORRIDOR_POLICIES:
            raise ValueError('typed G1/G2 entry and corridor policies required')
        reserve = self.declare_parameter('g1_g2_follow_entry_space',False).value
        moving_reserve = self.declare_parameter('g1_g2_moving_follow_reserve',False).value
        closing_reserve = self.declare_parameter('g1_g2_closing_follow_reserve',False).value
        recent_reserve = self.declare_parameter('g1_g2_recent_forward_reserve',False).value
        low_motion_reserve = self.declare_parameter('g1_g2_low_motion_reserve',False).value
        moving_pace_floor = self.declare_parameter('g1_g2_moving_pace_floor',False).value
        reacquire = self.declare_parameter('g1_g2_reacquire_policy','positive_command_v0').value
        preparation_hold = self.declare_parameter('g1_g2_preparation_hold','off').value
        initial_reference = self.declare_parameter('g1_g2_initial_reference','off').value
        motion_pause = self.declare_parameter('g1_g2_motion_pause_policy','off').value
        relative_yield = self.declare_parameter('g1_g2_relative_yield_policy','off').value
        if (type(reserve) is not bool or type(reacquire) is not str or type(preparation_hold) is not str
                or type(initial_reference) is not str or type(motion_pause) is not str):
            raise ValueError('typed G1/G2 follow space and reacquire options required')
        if (not self.g1_enabled or self.follow is None or self.r0_plus_enabled or self.m3_execution_enabled
                or not trace_path or not Path(trace_path).is_absolute()):
            raise ValueError('G1/G2 handoff requires G1 sensors, its own absolute trace and no competing controller')
        rear_sweep = self.declare_parameter('g1_g2_released_rear_sweep_trial',False).value
        observed_motion = self.declare_parameter('g1_g2_observed_motion_gates',False).value
        continuous_pace = self.declare_parameter('g1_g2_continuous_forward_pace',False).value
        moving_pass = self.declare_parameter('g1_g2_moving_pass_trial',False).value
        moving_profile = self.declare_parameter('g1_g2_moving_pass_profile','slow_v1').value
        wall_witness = self.declare_parameter('g1_g2_rear_wall_witness',False).value
        center_stopped = self.declare_parameter('g1_g2_stopped_entry_centering',False).value
        adaptive_preview = self.declare_parameter('g1_g2_adaptive_entry_preview',False).value
        if type(continuous_pace) is not bool:
            raise ValueError('typed G1/G2 continuous forward pace option required')
        side_departure = self.declare_parameter('g1_g2_side_departure_trial',False).value
        current_lane = self.declare_parameter('g1_g2_current_lane_preference_trial',False).value
        lane_headroom = self.declare_parameter('g1_g2_current_lane_headroom_mps',0.).value
        config = replace(self.follow.config,current_lane_preference_trial=current_lane,
                         current_lane_headroom_mps=lane_headroom,released_rear_sweep_trial=rear_sweep,static_side_release_trial=observed_motion,
                         side_departure_trial=side_departure)
        self.follow = BypassHandoffFollow(config,HandoffExecutionConfig(preparation_policy=policy,
            persistence_window_policy=window,follow_entry_space_enabled=reserve,moving_follow_reserve=moving_reserve,
            closing_follow_reserve=closing_reserve,recent_forward_reserve=recent_reserve,
            low_motion_reserve=low_motion_reserve,moving_pace_floor=moving_pace_floor,reacquire_policy=reacquire,
            entry_policy=entry_policy,preparation_hold_policy=preparation_hold,initial_reference_policy=initial_reference,
            motion_pause_policy=motion_pause,
            relative_yield_policy=relative_yield,entry_motion_gate=observed_motion,stopped_entry_centering=center_stopped,
            adaptive_entry_preview=adaptive_preview,
            observed_forward_pace=observed_motion and moving_pace_floor,
            continuous_forward_pace=continuous_pace,
            moving_pass_trial=moving_pass, moving_pass_profile=moving_profile,
            bypass=BypassExecutionConfig(corridor_policy=corridor,steerable_transition=observed_motion,
                                         rear_wall_witness=wall_witness)),
            allow_simulation_trial=True)
        trace = FollowTrace(trace_path,self.motion.history.config,self.follow.config,
            mode='g1_g2_handoff_trial',execution_config=self.follow.execution)
        self.get_logger().info('G1/G2 sensor handoff trial (preparation '+policy+'); /drive through independent safety')
        return trace

    def command(self, points):
        if self.decision_delivery is not None:
            self.async_inputs_ready = True
            item = self.decision_delivery.poll(self.now_s(), previous_steering_rad=self.last_steering,
                side=self.side, motion_epoch=self.motion.counts['resets'],
                sensor_epoch=getattr(self.scan, 'source_epoch', 0))
            if item is None:
                return None
            if item['kind'] == 'fault':
                return 0., self.last_steering, dict(reason='decision_process_fault', input_error=item['reason'])
            # 출력과 적용 여부를 분리한다. 버린 결과도 실제 계산/재생 증거로 보존한다.
            self.follow_trace.emit('call', dict(input=item['input'], output=item['result']['output'],
                delivery={k:v for k,v in item.items() if k not in ('input','result','output')},
                compute={k:v for k,v in item['result'].items() if k != 'output'}))
            if item['kind'] == 'discarded':
                self.follow.reset()
                self.follow_trace.emit('reset', dict(reason=item['reason']))
                return None
            self.decision_command_stamp_s = item['reference_s']
            target, steering, detail = self.prepared_output(item['output'])
            return target, steering, {**status_detail(detail), 'decision_delivery': {
                **self.decision_delivery.counts, 'source_age_s': item['source_age_s'],
                'reference_s': item['reference_s']}}
        target, steering, detail = super().command(points)
        return target, steering, status_detail(detail)


def main(args=None):
    run_node(G1G2Node,args=args)
