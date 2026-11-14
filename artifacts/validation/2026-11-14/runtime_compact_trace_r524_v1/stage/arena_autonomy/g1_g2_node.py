"""G1/G2 전환을 별도 실행 파일에서만 선택한다. 기존 launch의 기본값은 유지한다."""
from pathlib import Path
from arena_autonomy.vehicle_marker_channel import snapshot_options
from dataclasses import asdict, replace
import math
import time

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
    if 'vehicle_marker_occupancy' in detail:
        result['vehicle_marker_occupancy'] = detail['vehicle_marker_occupancy']
    return freeze(result)


class G1G2Node(LocalPursuit):
    # capability는 입구만 연다. 아래 native2slot/r396 실제 선택 검증은 계속 필수다.
    estimated_observation_runtime_capable = True
    metric_marker_runtime_capable = True
    def __init__(self):
        self.decision_delivery = None
        self.decision_notice = None
        self.decision_guard = None
        self.recheck_in_worker = False
        self.current_path_recheck = None
        self.runtime_dual_selected = False
        super().__init__()
        execution = self.declare_parameter('g1_decision_execution', 'inline').value
        if execution not in ('inline', 'process_candidate'):
            raise ValueError('registered decision execution required')
        submission = self.declare_parameter('g1_submission_policy', 'original_v1').value
        native_representation = 'native_estimated_v1' if self.hardware_inputs else 'native_simulation_v1'
        if submission not in ('original_v1', 'fresh_native_v1'):
            raise ValueError('registered decision submission policy required')
        if submission == 'fresh_native_v1' and (execution != 'process_candidate'
                or self.get_parameter('lidar_input_representation').value != native_representation):
            raise ValueError('fresh submission requires process_candidate and the selected native time domain')
        retain_reference = self.follow.config.expiry_route_reference_trial
        if retain_reference and (execution != 'process_candidate' or submission != 'fresh_native_v1'):
            raise ValueError('expiry reference requires process_candidate and fresh_native_v1')
        recheck = self.declare_parameter('g1_current_path_recheck', 'off').value
        if recheck not in ('off', 'selected_v1') or (recheck != 'off' and execution != 'process_candidate'):
            raise ValueError('current path recheck requires registered process_candidate selection')
        if recheck == 'selected_v1':
            from arena_autonomy import current_path_recheck
            self.current_path_recheck = current_path_recheck
            if self.follow.config.guard_observation_policy == 'selected_r181':
                from arena_autonomy import guard_observation_policy
                self.current_path_recheck = guard_observation_policy
        recheck_execution = self.declare_parameter('g1_current_recheck_execution', 'parent').value
        if recheck_execution not in ('parent', 'worker_event_v1'):
            raise ValueError('registered current recheck execution required')
        self.recheck_in_worker = recheck_execution == 'worker_event_v1'
        numeric_execution=self.declare_parameter('g1_numeric_execution','python').value
        if numeric_execution not in ('python','native_v1') or (numeric_execution=='native_v1' and not self.recheck_in_worker):
            raise ValueError('native numeric execution requires selected worker_event_v1')
        if self.recheck_in_worker and (execution != 'process_candidate' or submission != 'fresh_native_v1'
                or recheck != 'selected_v1' or self.follow.config.guard_observation_policy != 'selected_r181'):
            raise ValueError('worker recheck requires process/fresh native/selected observation policy')
        runtime_policy = self.declare_parameter('g1_runtime_policy', 'off').value
        from arena_autonomy.runtime_node_backend import validate_selection, validate_corner_selection, observation_options
        self.runtime_dual_selected = validate_selection(runtime_policy, execution=execution,
            submission=submission, recheck_in_worker=self.recheck_in_worker,
            retain_reference=retain_reference, numeric_execution=numeric_execution,
            hardware_shadow=self.hardware_shadow)
        self.complete_plan_trace = self.declare_parameter('g1_complete_plan_trace', False).value
        if self.vehicle_marker_channel.profile is not None and not self.runtime_dual_selected:
            raise ValueError('metric vehicle markers currently require registered dual plan/current runtime')
        if self.complete_plan_trace and not self.runtime_dual_selected:
            raise ValueError('complete plan trace requires the registered runtime policy')
        validate_corner_selection(self.current_corner_budget, runtime_selected=self.runtime_dual_selected,
            use_sim_time=self.get_parameter('use_sim_time').value, motion_verified=self.motion.verified,
            motion_time_policy=self.motion.timing_policy, **observation_options(self))
        if execution == 'process_candidate':
            if (not (self.get_parameter('use_sim_time').value is True or self.hardware_inputs)
                    or self.settings['control_rate_hz'] != 50.):
                raise ValueError('process candidate requires simulation or explicit hardware execution and a 50Hz parent')
            from arena_autonomy.decision_process import DecisionProcess
            from arena_autonomy.decision_delivery import DecisionDelivery
            delivery_type = DecisionDelivery
            worker_type = DecisionProcess
            if submission == 'fresh_native_v1':
                from arena_autonomy.fresh_submission import FreshStartDelivery
                delivery_type = FreshStartDelivery
            if self.recheck_in_worker:
                from arena_autonomy.recheck_process import TwoStageProcess
                from arena_autonomy.recheck_delivery import RecheckDelivery
                delivery_type, worker_type = RecheckDelivery, TwoStageProcess
            header = dict(schema=5, controller_mode='g1_g2_handoff_trial',
                motion_config=asdict(self.motion.history.config), follow_config=asdict(self.follow.config),
                execution_config=asdict(self.follow.execution))
            if numeric_execution!='python':header['numeric_execution']=numeric_execution
            delivery_options = dict(align_first_source=True)
            if retain_reference:
                delivery_options['retain_expired_reference'] = True
            if self.runtime_dual_selected:
                from arena_autonomy.runtime_node_backend import create_delivery
                self.decision_delivery = create_delivery(self, header)
                self.follow_trace.emit('runtime_factory_policy', dict(policy=runtime_policy,
                    backend='runtime_dual_v1', motion_time_policy=self.motion.timing_policy,
                    motion_timing_verified=self.motion.verified, physical_actuation_verified=False,
                    source_timestamps_preserved=True, default_changed=False,
                    **({'current_corner_budget': self.current_corner_budget}
                       if self.current_corner_budget != 'off' else {})))
            else:
                self.decision_delivery = delivery_type(worker_type(header), **delivery_options)
            if self.recheck_in_worker:
                from arena_autonomy.completion_notice import CompletionNotice
                self.decision_guard = self.create_guard_condition(self.on_decision_ready)
                self.decision_notice = CompletionNotice(self.decision_delivery.worker.connection,
                                                        self.decision_guard.trigger)
            self.last_request_time_s = -math.inf
            self.follow_trace.emit('reset', dict(reason='process_candidate_start',
                execution=execution, decision_dt_s=.1, receive_rate_hz=50., align_first_source=True))
            if self.hardware_shadow:
                self.follow_trace.emit('execution_environment', dict(mode='hardware_shadow_v1',
                    use_sim_time=False, drive_output=False, motion_timing_verified=self.motion.verified,
                    **({'motion_time_policy': self.motion.timing_policy}
                       if getattr(self.motion,'timing_policy','verified_only') != 'verified_only' else {})))
            if self.hardware_runtime:
                self.follow_trace.emit('execution_environment', dict(mode=self.execution_environment,
                    use_sim_time=False, motion_timing_verified=False,
                    motion_time_policy=self.motion.timing_policy,
                    command_requires_independent_guard=True, physical_actuation_verified=False))
            if submission == 'fresh_native_v1':
                if self.runtime_dual_selected:
                    self.follow_trace.emit('decision_submission_policy', dict(policy='runtime_dual_latest_v1',
                        minimum_reference_interval_s=.1, scan_timeout_s=.3, command_timeout_s=.2,
                        evaluation_delay_injection=False, source_timestamps_preserved=True))
                else:
                    self.follow_trace.emit('decision_submission_policy', dict(policy=submission,
                        completion_window_s=FreshStartDelivery.completion_window_s,
                        evaluation_delay_injection=False, source_timestamps_preserved=True))
            if self.current_path_recheck is not None:
                self.follow_trace.emit('current_path_recheck_policy', dict(policy=recheck,
                    independent_guard_required=True, evaluation_delay_injection=False,
                    **({'execution': recheck_execution,'numeric_execution':numeric_execution} if self.recheck_in_worker else {})))
            if retain_reference:
                self.follow_trace.emit('expiry_reference_policy', dict(policy='selected_wall_recent_v3',
                    command_retained=False, evaluation_delay_injection=False))

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
        self.trace_control(explicit_stop=True)

    def trace_control(self, *, explicit_stop=False):
        trace = getattr(self, 'follow_trace', None)
        if getattr(trace, 'detail', 'full') == 'compact_v1' and not trace.closed:
            trace.record_control(self.now_s(), 'EXPLICIT_STOP' if explicit_stop else self.last_status,
                self.last_speed, 0. if explicit_stop else self.last_steering,
                True if explicit_stop else self.command_was_published, explicit_stop=explicit_stop)

    def control(self):
        self.motion.drain_pending()
        if getattr(self, 'runtime_dual_selected', False):
            from arena_autonomy.runtime_node_backend import control
            result = control(self, super().control)
            self.trace_control()
            return result
        delivery = self.decision_delivery
        if delivery is None:
            result = super().control()
            self.trace_control()
            return result
        notice = getattr(self, 'decision_notice', None)
        if notice is not None and notice.fault:
            delivery.worker.fault = 'completion_notice: '+notice.fault
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
                    self.motion.verified, self.rear_blind_half_angle,
                    timing_policy=getattr(self.motion, 'timing_policy', 'verified_only'))
                delivery.offer(snapshot)
        self.trace_control()
        return result

    def on_decision_ready(self):
        notice = self.decision_notice
        if notice is None:
            return
        try:
            # 입력 부적합 때 command()가 호출되지 않아도 완료는 회수한다.
            # 결과 적용은 기존 control()의 신호/센서/발행 경계를 모두 거친다.
            self.decision_delivery.collect()
            if self.decision_delivery.worker.fault:
                notice.close()
                self.decision_notice = None
            self.control()
        finally:
            notice.acknowledge()

    def destroy_node(self):
        if self.decision_delivery is not None:
            if self.decision_notice is not None:
                report = self.decision_notice.close()
                if report['alive'] or report['fault']:
                    self.get_logger().warning('Decision notice cleanup: '+str(report))
                self.decision_notice = None
            if self.decision_guard is not None:
                self.destroy_guard_condition(self.decision_guard)
                self.decision_guard = None
            self.stop()
            cleanup = self.decision_delivery.worker.close()
            if cleanup and (cleanup['forced'] or cleanup['exitcode'] != 0):
                self.get_logger().warning('Decision process cleanup: '+str(cleanup))
        return super().destroy_node()

    def create_follow_trace(self, trace_path):
        trace_detail = self.declare_parameter('g1_trace_detail', 'full').value
        self.current_corner_budget = self.declare_parameter('g1_current_corner_budget', 'off').value
        if type(self.current_corner_budget) is not str or self.current_corner_budget not in ('off', 'r396_v1'):
            raise ValueError('registered current corner budget required')
        enabled = self.declare_parameter('g1_g2_simulation_trial',False).value
        if enabled is not True or not (self.get_parameter('use_sim_time').value is True or self.hardware_inputs):
            raise ValueError('G1/G2 handoff requires explicit trial activation and a registered execution environment')
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
            mode='g1_g2_handoff_trial',execution_config=self.follow.execution,
            current_corner_budget=self.current_corner_budget,
            observation_input_scope=getattr(self, 'observation_input_scope', 'verified_native_simulation_v1'),
            detail=trace_detail)
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
            if item['kind'] == 'needs_recheck':
                from arena_autonomy.follow_trace import snapshot_call
                now = self.motion.last_meta['reference_stamp_s']
                speed = self.motion.history.wheels.values[-1] if self.motion.history.wheels.values else 0.
                current = snapshot_call(self.scan, self.motion.history, now, self.motion.counts['resets'],
                    speed, self.side, self.last_steering, self.decision_output_dt_s,
                    self.motion.verified, self.rear_blind_half_angle,
                    timing_policy=getattr(self.motion, 'timing_policy', 'verified_only'),
                    **snapshot_options(self, now))
                self.decision_delivery.offer_recheck(current)
                return None
            # 출력과 적용 여부를 분리한다. 버린 결과도 실제 계산/재생 증거로 보존한다.
            evidence = dict(input=item['input'], output=item['result']['output'],
                delivery={k:v for k,v in item.items() if k not in ('input','result','output','recheck_input','recheck_result')},
                compute={k:v for k,v in item['result'].items() if k != 'output'})
            checked = item.get('recheck_result')
            if checked is not None:
                evidence['current_rebase'] = dict(input=item['recheck_input'], output=checked['output'],
                    computation_wall_ms=1000*(checked['completed_wall_s']-checked['began_wall_s']),
                    compute={k:v for k,v in checked.items() if k != 'output'})
            if item['kind'] == 'discarded':
                self.follow_trace.emit('call', evidence)
                self.follow.reset()
                self.follow_trace.emit('reset', dict(reason=item['reason']))
                return None
            output = item['output']
            reference = item.get('command_reference_s', item['reference_s'])
            current_result = None if checked is None else checked['output']
            recheck = getattr(self, 'current_path_recheck', None)
            if not getattr(self, 'recheck_in_worker', False) and recheck is not None and recheck.supported(output):
                from arena_autonomy.follow_trace import snapshot_call
                from arena_autonomy.lidar_motion import MotionUnavailable
                from arena_autonomy.scene_motion import motion_scene
                began = time.perf_counter()
                now = self.motion.last_meta['reference_stamp_s']
                speed = self.motion.history.wheels.values[-1] if self.motion.history.wheels.values else 0.
                current = snapshot_call(self.scan, self.motion.history, now, self.motion.counts['resets'],
                    speed, self.side, self.last_steering, self.decision_output_dt_s,
                    self.motion.verified, self.rear_blind_half_angle,
                    timing_policy=getattr(self.motion, 'timing_policy', 'verified_only'),
                    **snapshot_options(self, now))
                try:
                    scene = motion_scene(self.scan, self.motion.history, now,
                        timing_verified=self.motion.verified, epoch=self.motion.counts['resets'],
                        rear_half_angle_deg=self.rear_blind_half_angle,
                        timing_policy=getattr(self.motion, 'timing_policy', 'verified_only'))
                    current_result = recheck.rebase(output, item['input'], current, scene,
                                                   self.motion.history, self.follow.config)
                except (MotionUnavailable, ValueError) as error:
                    current_result = dict(speed_mps=0., steering_rad=self.last_steering,
                        reference_s=now, reason='rebase_input_stop', input_error=str(error),
                        independent_guard_required=True, runtime_authorized=False)
                if current_result is None:
                    raise RuntimeError('supported current recheck returned no result')
                detail = {**output[2], 'current_rebase': {
                    k:v for k,v in current_result.items() if k != 'path_points'}}
                if current_result['speed_mps'] <= 0:
                    detail['reason'] = current_result['reason']
                output = (current_result['speed_mps'], current_result['steering_rad'], detail)
                reference = current_result['reference_s']
                evidence['current_rebase'] = dict(input=current,
                    output=(current_result if getattr(self.follow_trace, 'detail', 'full') == 'compact_v1'
                            else freeze(current_result)),
                    computation_wall_ms=1000*(time.perf_counter()-began))
            self.follow_trace.emit('call', evidence)
            self.decision_command_stamp_s = reference
            target, steering, detail = self.prepared_output(output)
            compact = status_detail(detail)
            if current_result is not None:
                compact['current_rebase'] = detail['current_rebase']
            delivery_detail = {
                **self.decision_delivery.counts, 'source_age_s': item['source_age_s'],
                'reference_s': item['reference_s']}
            if current_result is not None:
                delivery_detail['command_reference_s'] = reference
            return target, steering, {**compact, 'decision_delivery': delivery_detail}
        target, steering, detail = super().command(points)
        return target, steering, status_detail(detail)


def main(args=None):
    run_node(G1G2Node,args=args)
