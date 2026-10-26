"""B0183 신호/분기 인식 + C1 지역 경로 추종. 기존 벽 추종과 선택 가능."""
import json
import time

from arena_autonomy.local_path import mask_scan, pursuit_command
from arena_autonomy.core import marker_ids
from arena_autonomy.wall_follow import WallFollow
from arena_autonomy.vision_process import VisionPipeline, VisionProcess
from arena_vehicle_interface.node_lifecycle import run_node
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionUnavailable


class LocalPursuit(WallFollow):
    def __init__(self):
        super().__init__("local_pursuit")
        self.rear_blind_half_angle = float(self.declare_parameter('rear_blind_half_angle_deg', 30.).value)
        if not 0 <= self.rear_blind_half_angle <= 80:
            raise ValueError('rear_blind_half_angle_deg must be within 0..80')
        if self.motion is None:
            raise ValueError("local_pursuit는 엔코더/IMU 운동 보정이 필요합니다.")
        self.g1_enabled = bool(self.declare_parameter('g1_follow_enabled', False).value)
        self.r0_plus_enabled = bool(self.declare_parameter('r0_plus_enabled', False).value)
        self.m3_execution_enabled = bool(self.declare_parameter('g1_m3_execution_trial', False).value)
        m3_command_fallback = bool(self.declare_parameter('g1_m3_command_fallback', False).value)
        if m3_command_fallback and not self.m3_execution_enabled:
            raise ValueError('M3 command fallback requires explicit M3 trial')
        m3_release_to_baseline = bool(self.declare_parameter('g1_m3_release_to_baseline', False).value)
        if m3_release_to_baseline and not self.m3_execution_enabled:
            raise ValueError('M3 baseline release requires explicit M3 trial')
        m3_small_center_straight = bool(self.declare_parameter('g1_m3_small_center_straight', False).value)
        if m3_small_center_straight and not self.m3_execution_enabled:
            raise ValueError('M3 small center straight requires explicit M3 trial')
        m3_surface_commitment = bool(self.declare_parameter('g1_m3_straight_surface_commitment', False).value)
        if m3_surface_commitment and not self.m3_execution_enabled:
            raise ValueError('M3 straight surface commitment requires explicit M3 trial')
        m3_departure_options = {name: bool(self.declare_parameter('g1_m3_'+name, False).value)
                                for name in ('rounded_occupancy', 'straight_with_threat', 'defer_center_on_straight')}
        if any(m3_departure_options.values()) and not self.m3_execution_enabled:
            raise ValueError('M3 departure options require explicit M3 trial')
        if self.g1_enabled and self.r0_plus_enabled:
            raise ValueError('G1 and R0+ are mutually exclusive')
        trace_path = str(self.declare_parameter('g1_trace_path', '').value)
        if self.m3_execution_enabled and (not self.g1_enabled or self.r0_plus_enabled
                or not self.get_parameter('use_sim_time').value or not trace_path):
            raise ValueError('M3 execution trial requires explicit G1, simulation time and decision recording')
        boundary_mode = str(self.declare_parameter('g1_boundary_evidence_mode', 'connected').value)
        occupancy_mode = str(self.declare_parameter('g1_occupancy_timing_mode', 'oldest').value)
        corridor_fallback = bool(self.declare_parameter('g1_corridor_fallback_enabled', False).value)
        recent_observation = bool(self.declare_parameter('g1_recent_observation_enabled', False).value)
        if self.m3_execution_enabled and (occupancy_mode != 'per_ray_candidate' or recent_observation):
            raise ValueError('M3 execution trial requires per-ray occupancy and no recent observation')
        if recent_observation and not self.g1_enabled:
            raise ValueError('recent observation requires explicit G1 activation')
        if corridor_fallback and not self.g1_enabled:
            raise ValueError('corridor fallback requires explicit G1 activation')
        if occupancy_mode not in ('oldest', 'per_ray_candidate') or (not (self.g1_enabled or self.r0_plus_enabled) and occupancy_mode != 'oldest'):
            raise ValueError('per-ray occupancy candidate requires G1')
        if boundary_mode not in ('connected', 'fragmented_candidate', 'corner_anchor_candidate', 'corner_distance_candidate') or (not self.g1_enabled and boundary_mode != 'connected'):
            raise ValueError('boundary candidate requires G1')
        if (self.g1_enabled or self.r0_plus_enabled) and (self.motion.mode != 'both' or not self.motion.verified):
            raise ValueError('G1은 검증된 순차 취득과 both 운동 보정이 필요합니다.')
        follow_type = LocalFollow
        if self.r0_plus_enabled:
            from arena_autonomy.r0_plus import R0Plus
            follow_type = R0Plus
        self.follow = follow_type(FollowConfig(
            corridor_fallback_enabled=corridor_fallback,
            recent_observation_enabled=recent_observation,
            boundary_evidence_mode=boundary_mode,
            occupancy_timing_mode=occupancy_mode,
            wheelbase_m=float(self.settings['wheelbase_m']),
            max_steering_rad=float(self.settings['max_steering_angle_rad']),
            max_speed_mps=min(1., float(self.settings['max_speed_mps'])),
            lateral_acceleration_mps2=float(self.settings['lateral_acceleration_limit_mps2']),
            wall_offset_m=float(self.settings['target_wall_distance_m']))) if (self.g1_enabled or self.r0_plus_enabled) else None
        if self.m3_execution_enabled:
            from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
            self.follow = M3ExecutionFollow(self.follow.config,
                ExecutionConfig(command_fallback=m3_command_fallback, release_to_baseline=m3_release_to_baseline,
                                small_center_straight=m3_small_center_straight,
                                straight_surface_commitment=m3_surface_commitment, **m3_departure_options),
                allow_simulation_trial=True)
        self.vision_execution = str(self.declare_parameter('vision_execution', 'process').value)
        if self.vision_execution not in ('inline', 'process'):
            raise ValueError('vision_execution must be inline or process')
        self.start_signal_mode = str(self.declare_parameter('start_signal_mode', 'temporary_3light').value)
        if self.start_signal_mode not in ('temporary_3light', 'official_4red_candidate'):
            raise ValueError('unsupported start_signal_mode')
        if self.start_signal_mode != 'temporary_3light' and self.vision_execution != 'process':
            raise ValueError('official start candidate requires process vision')
        self.race_control_mode = str(self.declare_parameter('race_control_mode', 'legacy').value)
        if self.race_control_mode not in ('legacy', 'b3_candidate'):
            raise ValueError('unsupported race_control_mode')
        if self.race_control_mode != 'legacy' and self.vision_execution != 'process':
            raise ValueError('B3 race supervisor requires process vision generations')
        self.vision_pipeline = None
        if self.vision_execution == 'process':
            self.vision_pipeline = VisionPipeline(VisionProcess(),
                image_timeout=self.settings['image_timeout_s'],
                wall_timeout=self.settings['sensor_wall_timeout_s'], signal_mode=self.start_signal_mode)
        self.race = None
        if self.race_control_mode == 'b3_candidate':
            # 기본 legacy에서는 이 구독/발행을 만들지 않는다. MCU 상태는 판정 입력이며 명령을 보내지 않는다.
            from std_msgs.msg import String
            from arena_autonomy.race_session import McuObserver, RaceSupervisor
            self.race = RaceSupervisor(self.vision_pipeline, image_max_age_s=self.settings['image_timeout_s'],
                mcu=McuObserver(wall_timeout_s=self.settings['sensor_wall_timeout_s']), now_s=self.now_s())
            self.race_result_publisher = self.create_publisher(String, '/race/command_result', 10)
            self.create_subscription(String, '/actuation/status', self.on_mcu_status, 10)
            self.create_subscription(String, '/race/command', self.on_race_command, 10)
            self.get_logger().info('B3 race supervisor candidate: session='+self.race.session)
        self.follow_trace = None
        if trace_path:
            if self.follow is None:
                raise ValueError('decision trace requires explicit G1 or R0+ activation')
            from arena_autonomy.follow_trace import FollowTrace, controller_mode
            self.follow_trace = FollowTrace(trace_path, self.motion.history.config, self.follow.config,
                mode=controller_mode(self.follow), execution_config=getattr(self.follow, 'execution', None))

    def reset_follow(self, reason):
        self.follow.reset()
        if getattr(self, 'follow_trace', None) is not None:
            self.follow_trace.emit('reset', {'reason': reason})

    def on_image(self, message):
        if self.vision_pipeline is None:
            return super().on_image(message)
        self.vision_pipeline.offer(message)

    def on_reset(self, request, response):
        if getattr(self, 'race', None) is not None:
            # 구형 reset은 B3 래치를 풀 수 없다. 새 경기와 새 개입 래치를 함께 적용한다.
            response = super().on_reset(request, response)
            self.race_reset(lambda: self.race.legacy_reset(self.now_s()), 'legacy_reset')
            response.message = ('B3: new race and latched intervention %d; acknowledge, MCU re-ARM and '
                                'new visual sequence required' % self.race.permission.intervention)
            return response
        if getattr(self, 'follow', None) is not None:
            self.reset_follow('autonomy_reset')
        if self.vision_pipeline is not None:
            self.vision_pipeline.reset()
        return super().on_reset(request, response)

    def race_reset(self, operation, reason):
        """경기/영상 세대와 계획기 기억·신호 참조를 같은 콜백에서 바꾼다."""
        result = operation()
        if getattr(self, 'follow', None) is not None:
            self.reset_follow(reason)
        self.signal = self.vision_pipeline.signal
        self.ids = []
        self.stop()
        return result

    def on_enable(self, request, response):
        if getattr(self, 'race', None) is None:
            return super().on_enable(request, response)
        if request.data:
            response.success = False
            response.message = 'B3: enable cannot release a race latch; use /race/command acknowledge'
            return response
        token = self.race.stop('autonomy_enable_false', self.now_s())
        self.stop()
        response.success = True
        response.message = 'B3 policy stop latched: intervention %d (MCU STOP is separate)' % token
        return response

    def on_mcu_status(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            data = None
        self.race.observe_mcu(data, self.now_s(), time.monotonic())

    def on_race_command(self, message):
        from std_msgs.msg import String
        try:
            payload = json.loads(message.data)
        except ValueError:
            payload = None
        previous = self.race.permission.race_epoch
        result = self.race.handle_command(payload, self.now_s())
        if result['accepted'] and self.race.permission.race_epoch != previous:
            # handle_command가 경기/영상 세대를 바꿨다. 계획기 기억과 신호 참조도 같은 콜백에서 정리한다.
            self.race_reset(lambda: None, 'new_race')
        elif result['accepted'] and result['op'] == 'stop':
            self.stop()
        self.race_result_publisher.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))

    def motion_gate(self, status):
        """B3 선택형에서만 호출된다. 허가는 /drive 요청 상한이며 독립 보호층·MCU가 따로 거절한다."""
        decision = self.race.decide(sensors_ready=True, now_s=status['sim_time_s'], wall_s=time.monotonic())
        if not decision['allowed']:
            status['state'] = {'wait_visual_start': 'WAIT_GREEN',
                               'actuator_not_ready': 'ACTUATOR_WAIT'}.get(decision['reason'], 'RACE_LATCHED')
        return decision['allowed']

    def control(self):
        if self.vision_pipeline is not None:
            # 시계가 되감겼으면 결과를 읽기 전에 이전 세대를 무효화한다.
            if self.now_s() < self.last_control_time:
                from std_srvs.srv import Trigger
                if getattr(self, 'race', None) is not None:
                    from arena_autonomy.wall_follow import WallFollow
                    WallFollow.on_reset(self, None, Trigger.Response())
                    self.race_reset(lambda: self.race.clock_reset(self.now_s()), 'clock_reset')
                else:
                    was_enabled = self.enabled
                    self.on_reset(None, Trigger.Response())
                    self.enabled = was_enabled
                self.last_control_time = float('-inf')
            result = self.timing.call('vision_poll', self.vision_pipeline.pump, self.now_s())
            if getattr(self, 'race', None) is not None:
                self.race.poll_faults(self.now_s())
                if result is not None and not self.vision_pipeline.fault:
                    self.race.observe_vision(result, self.vision_pipeline.epoch, self.now_s())
            if self.vision_pipeline.fault:
                self.rgb_stamp = None
            elif result is not None:
                self.signal = self.vision_pipeline.signal
                self.ids = list(self.vision_pipeline.ids)
                self.rgb_stamp = result['stamp']
                self.image_wall_time = result['received']
                if 30 in self.ids:
                    self.side = 'right'
                elif any(marker in self.ids for marker in (0, 20, 45)):
                    self.side = 'left'
        result = super().control()
        if getattr(self, 'follow', None) is not None and self.last_status in (
                'WAIT_GREEN', 'DISABLED', 'SENSOR_STOP', 'MOTION_STOP', 'SCAN_INVALID',
                'RACE_LATCHED', 'ACTUATOR_WAIT'):
            self.reset_follow(self.last_status)
        return result

    def destroy_node(self):
        # 영상 정리를 기다리기 전에 주행 출력은 먼저 정지한다.
        self.stop()
        if getattr(self, 'follow_trace', None) is not None:
            try:
                self.follow_trace.close()
            except Exception as error:
                self.get_logger().warning('G1 trace close failed: '+str(error))
        if self.vision_pipeline is not None:
            cleanup = self.vision_pipeline.close()
            if cleanup and (cleanup['forced'] or cleanup['exitcode'] != 0):
                self.get_logger().warning('Vision process cleanup: '+str(cleanup))
        return super().destroy_node()

    def on_scan(self, message):
        super().on_scan(mask_scan(message, self.rear_blind_half_angle))

    def detect_markers(self, rgb):
        return sorted(set(marker_ids(rgb, .02)))

    def command(self, points):
        # 명령 속도가 아니라 엔코더 관측으로 선행 거리를 정한다.
        speed = self.motion.history.wheels.values[-1] if self.motion.history.wheels.values else 0.
        if getattr(self, 'follow', None) is not None:
            try:
                scene = motion_scene(self.scan, self.motion.history,
                    self.motion.last_meta['reference_stamp_s'], timing_verified=self.motion.verified,
                    epoch=self.motion.counts['resets'], rear_half_angle_deg=self.rear_blind_half_angle)
                output = self.follow.command(scene, self.motion.history, speed, self.side,
                                             self.last_steering, 1/self.settings['control_rate_hz'])
            except (MotionUnavailable, ValueError) as error:
                self.follow.reset()
                from arena_autonomy.follow_trace import controller_mode
                output = 0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD',
                                 'input_error': str(error), 'mode': controller_mode(self.follow)}
            if getattr(self, 'follow_trace', None) is not None:
                self.follow_trace.record_call(self.scan, self.motion.history,
                    self.motion.last_meta.get('reference_stamp_s'), self.motion.counts['resets'],
                    speed, self.side, self.last_steering, 1/self.settings['control_rate_hz'],
                    self.motion.verified, self.rear_blind_half_angle, output)
            return output
        return pursuit_command(points, self.side, speed,
                               self.settings["wheelbase_m"], self.settings["max_steering_angle_rad"],
                               self.settings["max_speed_mps"],
                               self.settings["lateral_acceleration_limit_mps2"],
                               self.settings["target_wall_distance_m"],
                               self.motion.last_meta.get('oldest_observation_age_s', .2))


def main(args=None):
    run_node(LocalPursuit, args=args)
