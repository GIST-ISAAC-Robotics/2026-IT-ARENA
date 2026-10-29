"""ToF 없는 구성의 독립 라이다 보호층. /drive -> /drive/safe."""
import json
import math
import time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lidar_motion_ros import MotionInput, seconds
from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.local_path import mask_scan, swept_limit, braking_speed
from arena_vehicle_interface.node_lifecycle import run_node
from arena_vehicle_interface.timing_probe import TimingProbe


class LidarSafety(Node):
    def __init__(self):
        super().__init__("lidar_safety")
        self.timing = TimingProbe(self)
        self.motion = MotionInput(self)
        self.wall_timeout = float(self.declare_parameter("sensor_wall_timeout_s", 3.).value)
        self.limit = float(self.declare_parameter("max_steering_angle_rad", .37).value)
        self.wheelbase = float(self.declare_parameter('wheelbase_m', .145).value)
        self.length = float(self.declare_parameter('vehicle_length_m', .20).value)
        self.width = float(self.declare_parameter('vehicle_width_m', .15).value)
        self.rear_blind_half_angle = float(self.declare_parameter('rear_blind_half_angle_deg', 30.).value)
        if not all(math.isfinite(v) and v > 0 for v in (self.wall_timeout, self.limit, self.wheelbase, self.length, self.width)):
            raise ValueError('LiDAR safety geometry and timeouts must be finite and positive')
        if not 0 <= self.rear_blind_half_angle <= 80:
            raise ValueError('rear_blind_half_angle_deg must be within 0..80')
        self.recent_observation_enabled = bool(self.declare_parameter('g1_recent_observation_enabled', False).value)
        self.partial_policy = None
        policy = str(self.declare_parameter('partial_observation_policy', 'strict').value)
        traffic_recovery = bool(self.declare_parameter('g1_traffic_recovery_trial', False).value)
        if traffic_recovery and policy != 'bounded_local_v4':
            raise ValueError('traffic recovery safety requires explicit v4 policy')
        if policy not in ('strict', 'bounded_side_v1', 'bounded_local_v2', 'bounded_local_v3', 'bounded_local_v4'):
            raise ValueError('unknown partial observation policy')
        if policy != 'strict':
            if self.recent_observation_enabled or self.motion.mode != 'both' or not self.motion.verified:
                raise ValueError('bounded observation policy requires verified both motion and no recent candidate')
            from arena_autonomy.partial_observation import PartialObservationPolicy
            if traffic_recovery:
                from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
                PartialObservationPolicy = RecoveryPolicy
            self.partial_policy = PartialObservationPolicy(allow_narrow_forward=policy in ('bounded_local_v2','bounded_local_v3','bounded_local_v4'),
                                                          surface_translation_enabled=policy in ('bounded_local_v3','bounded_local_v4'),
                                                          lateral_metric_enabled=policy=='bounded_local_v4')
        self.observation_cache = None
        self.observation_epoch = 0
        self.previous_safe_steering = 0.
        if self.recent_observation_enabled:
            from arena_autonomy.local_follow import FollowConfig
            from arena_autonomy.observation_support import RecentScenes
            enabled = bool(self.declare_parameter('g1_follow_enabled', False).value)
            corridor = bool(self.declare_parameter('g1_corridor_fallback_enabled', False).value)
            if not enabled or not corridor or self.motion.mode!='both' or not self.motion.verified:
                raise ValueError('recent observation safety requires G1, corridor, verified sequential both motion')
            self.observation_config = FollowConfig(recent_observation_enabled=True, corridor_fallback_enabled=True,
                boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
                wheelbase_m=self.wheelbase, length_m=self.length, width_m=self.width, max_steering_rad=self.limit)
            self.observation_cache = RecentScenes()
        self.scan = self.command = None
        self.scan_wall = self.command_wall = 0.
        self.last_clock = None
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive/safe", 10)
        self.status = self.create_publisher(String, "/safety/status", 10)
        self.create_subscription(LaserScan, "/scan", self.timing.wrap('scan', self.on_scan), qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, "/drive", self.timing.wrap('command', self.on_command), 10)
        self.create_timer(.02, self.timing.wrap('control', self.control))

    def on_scan(self, msg):
        if self.scan is None or seconds(msg.header.stamp) >= seconds(self.scan.header.stamp):
            self.scan, self.scan_wall = mask_scan(msg, self.rear_blind_half_angle), time.monotonic()

    def on_command(self, msg):
        if self.command is None or seconds(msg.header.stamp) >= seconds(self.command.header.stamp):
            self.command, self.command_wall = msg, time.monotonic()

    def control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.scan = self.command = None
            self.motion.history.reset()
            if getattr(self, 'partial_policy', None) is not None:
                self.partial_policy.reset()
            if getattr(self,'observation_cache',None) is not None:
                self.observation_cache.reset()
                self.observation_epoch += 1
                self.previous_safe_steering = 0.
        self.last_clock = now
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link"
        if self.command is not None and math.isfinite(self.command.drive.steering_angle):
            output.drive.steering_angle = float(np.clip(self.command.drive.steering_angle, -self.limit, self.limit))
        reason, clearance = "input_stop", 0.
        observation = None
        recent = None
        partial = None
        try:
            if self.scan is None or self.command is None:
                raise MotionUnavailable("missing_input")
            if (not 0 <= now - seconds(self.command.header.stamp) <= .2 or
                not 0 <= now - seconds(self.scan.header.stamp) <= .3 or
                max(time.monotonic() - self.scan_wall, time.monotonic() - self.command_wall) > self.wall_timeout):
                raise MotionUnavailable("stale_input")
            speed, steering = self.command.drive.speed, self.command.drive.steering_angle
            if not all(math.isfinite(v) for v in (speed, steering)) or speed < 0:
                raise MotionUnavailable("invalid_or_reverse_command")
            # Slamtec 드라이버의 +Inf는 raw 0/무응답도 포함한다. Gazebo의 광선
            # 미교차와 구별할 관측 증거가 없으면 어느 쪽도 자유 공간으로 쓰지 않는다.
            observation = audit_front_observation(
                self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                self.scan.range_min, self.scan.range_max,
                self.motion.history.config.lidar_yaw_rad)
            candidate = getattr(self,'recent_observation_enabled',False)
            partial_policy = getattr(self, 'partial_policy', None)
            if observation.reason != 'ok' and not ((candidate or partial_policy is not None) and observation.reason=='front_unobserved'):
                raise MotionUnavailable(observation.reason)
            points, count = self.motion.project(self.scan)
            if count < 30:
                raise MotionUnavailable("scan_invalid")
            if partial_policy is not None:
                stamps, values = self.motion.history.wheels.available(now)
                if not len(stamps) or now-stamps[-1] > self.motion.history.config.max_extrapolation_s+1e-9:
                    raise MotionUnavailable('motion_stale')
                measured = float(values[-1])
                if not math.isfinite(measured):
                    raise MotionUnavailable('invalid_or_reverse_feedback')
                # 역회전 피드백은 출력을 막되, 새 스캔의 관측 기억까지 끊지 않는다.
                # 제동 때 모터축 값이 왕복하면 실제 센서는 살아 있는데 기억만 만료될 수 있다.
                # 운동 보정에는 원래 부호를 유지하고, 외곽 크기에만 속력 상계를 사용한다.
                partial = partial_policy.evaluate(self.scan, points, self.motion.history, now, max(speed, abs(measured)),
                    output.drive.steering_angle, getattr(self,'previous_safe_steering',0.),
                    self.wheelbase,self.length,self.width)
                if measured < 0:
                    raise MotionUnavailable('invalid_or_reverse_feedback')
                if not partial['allowed']:
                    raise MotionUnavailable(partial['reason'])
                if partial['speed_cap_mps'] is not None:
                    speed = min(speed, partial['speed_cap_mps'])
            if candidate:
                from arena_autonomy.observation_support import needs_recent_support, sweep_support, side_missing_candidate
                from arena_autonomy.scene_motion import motion_scene
                scene = motion_scene(self.scan,self.motion.history,now,timing_verified=self.motion.verified,
                                     epoch=self.observation_epoch,rear_half_angle_deg=self.rear_blind_half_angle)
                required = observation.reason!='ok' or needs_recent_support(scene)
                sources, cache_reason = self.observation_cache.observe(scene,self.motion.history,
                                                                      transport=required and speed>0)
                recent = {'required':required,'cache_reason':cache_reason,'rear_dynamic_risk_unresolved':True}
                if required:
                    if not side_missing_candidate(scene):
                        raise MotionUnavailable('front_unobserved')
                    if speed>0:
                        stamps, values = self.motion.history.wheels.available(now)
                        if not len(stamps) or now-stamps[-1]>self.motion.history.config.max_extrapolation_s+1e-9:
                            raise MotionUnavailable('motion_stale')
                        motion = self.motion.history.config
                        # 계획기의 지지 결과/경로는 받지 않는다. 독립 입력으로 다시 계산한다.
                        recent = sweep_support(sources,float(values[-1]),min(.2,speed),output.drive.steering_angle,
                            self.previous_safe_steering,self.observation_config,
                            protected_sensor_pose=(motion.lidar_x_m,motion.lidar_y_m,motion.lidar_yaw_rad))
                        if not recent['supported']:
                            raise MotionUnavailable('observation_unconfirmed')
                        speed = min(.2,speed)
            output.drive.steering_angle = float(np.clip(steering, -self.limit, self.limit))
            clearance = swept_limit(points, output.drive.steering_angle, self.wheelbase,
                                    length=self.length, width=self.width)
            cap = braking_speed(clearance, now - seconds(self.scan.header.stamp))
            output.drive.speed = float(min(speed, cap))
            reason = "clear" if cap >= speed else "lidar_braking"
        except MotionUnavailable as error:
            reason = str(error)
        if getattr(self,'recent_observation_enabled',False) or getattr(self,'partial_policy',None) is not None:
            self.previous_safe_steering = output.drive.steering_angle
        self.publisher.publish(output)
        self.status.publish(String(data=json.dumps({"reason": reason, "safe_speed_mps": output.drive.speed,
                                                    "swept_clearance_m": clearance, "sensor": "C1",
                                                    "no_return_policy": "unknown",
                                                    "front_observation": vars(observation) if observation else None,
                                                    **({'partial_observation':partial} if getattr(self,'partial_policy',None) is not None else {}),
                                                    **({'recent_observation':recent} if getattr(self,'recent_observation_enabled',False) else {})})))


def main(args=None):
    run_node(LidarSafety, args=args)
