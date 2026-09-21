"""B0183 신호/분기 인식 + C1 지역 경로 추종. 기존 벽 추종과 선택 가능."""
from arena_autonomy.local_path import mask_scan, pursuit_command
from arena_autonomy.core import marker_ids
from arena_autonomy.wall_follow import WallFollow
from arena_vehicle_interface.node_lifecycle import run_node


class LocalPursuit(WallFollow):
    def __init__(self):
        super().__init__("local_pursuit")
        self.rear_blind_half_angle = float(self.declare_parameter('rear_blind_half_angle_deg', 30.).value)
        if not 0 <= self.rear_blind_half_angle <= 80:
            raise ValueError('rear_blind_half_angle_deg must be within 0..80')
        if self.motion is None:
            raise ValueError("local_pursuit는 엔코더/IMU 운동 보정이 필요합니다.")

    def on_scan(self, message):
        super().on_scan(mask_scan(message, self.rear_blind_half_angle))

    def detect_markers(self, rgb):
        return sorted(set(marker_ids(rgb, .02)))

    def command(self, points):
        # 명령 속도가 아니라 엔코더 관측으로 선행 거리를 정한다.
        speed = self.motion.history.wheels.values[-1] if self.motion.history.wheels.values else 0.
        return pursuit_command(points, self.side, speed,
                               self.settings["wheelbase_m"], self.settings["max_steering_angle_rad"],
                               self.settings["max_speed_mps"],
                               self.settings["lateral_acceleration_limit_mps2"],
                               self.settings["target_wall_distance_m"],
                               self.motion.last_meta.get('oldest_observation_age_s', .2))


def main(args=None):
    run_node(LocalPursuit, args=args)
