"""B0183 신호/분기 인식 + C1 지역 경로 추종. 기존 벽 추종과 선택 가능."""
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
        if self.g1_enabled and (self.motion.mode != 'both' or not self.motion.verified):
            raise ValueError('G1은 검증된 순차 취득과 both 운동 보정이 필요합니다.')
        self.follow = LocalFollow(FollowConfig(
            wheelbase_m=float(self.settings['wheelbase_m']),
            max_steering_rad=float(self.settings['max_steering_angle_rad']),
            max_speed_mps=min(1., float(self.settings['max_speed_mps'])),
            lateral_acceleration_mps2=float(self.settings['lateral_acceleration_limit_mps2']),
            wall_offset_m=float(self.settings['target_wall_distance_m']))) if self.g1_enabled else None
        self.vision_execution = str(self.declare_parameter('vision_execution', 'process').value)
        if self.vision_execution not in ('inline', 'process'):
            raise ValueError('vision_execution must be inline or process')
        self.vision_pipeline = None
        if self.vision_execution == 'process':
            self.vision_pipeline = VisionPipeline(VisionProcess(),
                image_timeout=self.settings['image_timeout_s'],
                wall_timeout=self.settings['sensor_wall_timeout_s'])

    def on_image(self, message):
        if self.vision_pipeline is None:
            return super().on_image(message)
        self.vision_pipeline.offer(message)

    def on_reset(self, request, response):
        if getattr(self, 'follow', None) is not None:
            self.follow.reset()
        if self.vision_pipeline is not None:
            self.vision_pipeline.reset()
        return super().on_reset(request, response)

    def control(self):
        if self.vision_pipeline is not None:
            # 시계가 되감겼으면 결과를 읽기 전에 이전 세대를 무효화한다.
            if self.now_s() < self.last_control_time:
                from std_srvs.srv import Trigger
                was_enabled = self.enabled
                self.on_reset(None, Trigger.Response())
                self.enabled = was_enabled
                self.last_control_time = float('-inf')
            result = self.timing.call('vision_poll', self.vision_pipeline.pump, self.now_s())
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
                'WAIT_GREEN', 'DISABLED', 'SENSOR_STOP', 'MOTION_STOP', 'SCAN_INVALID'):
            self.follow.reset()
        return result

    def destroy_node(self):
        # 영상 정리를 기다리기 전에 주행 출력은 먼저 정지한다.
        self.stop()
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
                return self.follow.command(scene, self.motion.history, speed, self.side,
                                           self.last_steering, 1/self.settings['control_rate_hz'])
            except (MotionUnavailable, ValueError) as error:
                self.follow.reset()
                return 0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD',
                               'input_error': str(error), 'mode': 'g1_follow'}
        return pursuit_command(points, self.side, speed,
                               self.settings["wheelbase_m"], self.settings["max_steering_angle_rad"],
                               self.settings["max_speed_mps"],
                               self.settings["lateral_acceleration_limit_mps2"],
                               self.settings["target_wall_distance_m"],
                               self.motion.last_meta.get('oldest_observation_age_s', .2))


def main(args=None):
    run_node(LocalPursuit, args=args)
