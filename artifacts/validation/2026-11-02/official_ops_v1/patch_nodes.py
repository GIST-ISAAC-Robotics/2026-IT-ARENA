"""CRLF 작업본을 유지한 채 제어 노드 두 파일을 고친다. 재실행 금지(치환 실패 시 중단)."""
from pathlib import Path


def patch(path, pairs):
    target = Path(path)
    raw = target.read_bytes()
    crlf = raw.count(b'\r\n') == raw.count(b'\n')
    text = raw.replace(b'\r\n', b'\n').decode('utf-8')
    for old, new in pairs:
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    data = text.encode('utf-8')
    target.write_bytes(data.replace(b'\n', b'\r\n') if crlf else data)


patch('src/arena_autonomy/arena_autonomy/local_pursuit.py', [
("""import json
import time

from arena_autonomy.local_path import mask_scan, pursuit_command
""", """import json
import math
import time

from arena_autonomy.local_path import mask_scan, pursuit_command
"""),
("""from arena_autonomy.core import marker_ids
from arena_autonomy.wall_follow import WallFollow
""", """from arena_autonomy.core import marker_ids
from arena_autonomy.wall_follow import WallFollow, stamp_seconds
"""),
("""class LocalPursuit(WallFollow):
    def __init__(self):
""", """# 경로는 맞지만 실제 앞차·점유 때문에 멈춘 판단. 경로 계획/독립 보호가 처리할 상황이므로 재개를 막지 않는다.
ROUTE_HOLDS = ('follow_gap_hold',)


class LocalPursuit(WallFollow):
    # 같은 경기 재개의 입력 경계. 수신 시각이 아니라 원천 취득 시각으로 비교한다.
    input_floor_s = -math.inf
    resume_route = None
    side_marker = None

    def __init__(self):
"""),
("""            self.create_subscription(String, '/race/command', self.on_race_command, 10)
""", """            self.create_subscription(String, '/race/command', self.on_race_command, 10)
            self.create_subscription(String, '/race/operator_link', self.on_operator_link, 10)
            self.input_boundary_publisher = self.create_publisher(String, '/race/input_boundary', 10)
"""),
("""        self.signal = self.vision_pipeline.signal
        self.ids = []
        self.stop()
        return result

    def on_enable(self, request, response):
""", """        self.signal = self.vision_pipeline.signal
        self.ids = []
        self.input_floor_s = -math.inf
        self.resume_route = None
        self.stop()
        return result

    def begin_input_boundary(self, boundary_s, reason):
        \"\"\"같은 경기 입력 경계. 경기/출발 사실과 논리적 기준 벽은 두고 기하·운동·영상 이력만 버린다.

        영상 세대는 RaceSupervisor가 같은 호출에서 올린다. 경계 전 취득 입력은 늦게 도착해도 쓰지 않는다.
        \"\"\"
        from std_msgs.msg import String
        self.input_floor_s = boundary_s
        self.scan, self.scan_wall_time = None, 0.
        self.rgb_stamp, self.image_wall_time = None, 0.
        self.ids = []
        self.resume_route = None
        self.motion.begin_input_boundary(boundary_s)
        if getattr(self, 'follow', None) is not None:
            self.reset_follow(reason)
        self.stop()
        self.input_boundary_publisher.publish(String(data=json.dumps(dict(boundary_s=boundary_s, reason=reason))))

    def on_enable(self, request, response):
"""),
("""        result = self.race.handle_command(payload, self.now_s())
""", """        result = self.race.handle_command(payload, self.now_s(), time.monotonic())
"""),
("""            self.race_reset(lambda: None, 'new_race')
        elif result['accepted'] and result['op'] == 'stop':
            self.stop()
        self.race_result_publisher.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))

    def motion_gate(self, status):
        \"\"\"B3 선택형에서만 호출된다. 허가는 /drive 요청 상한이며 독립 보호층·MCU가 따로 거절한다.\"\"\"
        decision = self.race.decide(sensors_ready=True, now_s=status['sim_time_s'], wall_s=time.monotonic())
""", """            self.race_reset(lambda: None, 'new_race')
        elif result['accepted'] and result['op'] == 'stop':
            self.stop()
        elif result['accepted'] and result['op'] in ('recovery_reset', 'resume_start', 'restore_race'):
            if result['op'] == 'restore_race':
                self.side, self.side_marker = result['side'], None
            self.begin_input_boundary(result['boundary_s'], result['op'])
        self.race_result_publisher.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))

    def on_operator_link(self, message):
        self.race.operator_link(message.data, time.monotonic())

    def resume_inputs(self, now):
        \"\"\"START 경계 뒤 원천 시각의 입력만 준비 완료로 센다. 영상은 처리까지 끝난 프레임만 인정한다.\"\"\"
        boundary = self.race.resume['boundary_s']
        history = self.motion.history
        return dict(scan=self.scan is not None and boundary <= stamp_seconds(self.scan.header.stamp)
                         and now-stamp_seconds(self.scan.header.stamp) < self.settings['scan_timeout_s'],
                    rgb=self.rgb_stamp is not None and self.rgb_stamp >= boundary,
                    gyro=bool(len(history.gyro.times)) and history.gyro.times[-1] >= boundary,
                    encoder=bool(len(history.wheels.times)) and history.wheels.times[-1] >= boundary)

    def input_snapshot(self, now):
        \"\"\"표시용 입력 나이. 판정에 쓰지 않으며 값이 없으면 None으로 남긴다.\"\"\"
        history = self.motion.history
        scan_stamp = None if self.scan is None else stamp_seconds(self.scan.header.stamp)
        gyro = history.gyro.times[-1] if len(history.gyro.times) else None
        encoder = history.wheels.times[-1] if len(history.wheels.times) else None
        ages = dict(scan=scan_stamp, rgb=self.rgb_stamp, gyro=gyro, encoder=encoder)
        return dict(age_s={key: None if value is None else now-value for key, value in ages.items()},
                    floor_s=None if math.isinf(self.input_floor_s) else self.input_floor_s,
                    side_marker=self.side_marker, route=self.resume_route,
                    gyro_rejected=self.motion.last_gyro_rejection)

    def motion_gate(self, status):
        \"\"\"B3 선택형에서만 호출된다. 허가는 /drive 요청 상한이며 독립 보호층·MCU가 따로 거절한다.\"\"\"
        resume = getattr(self.race, 'resume', None)
        if resume is not None:
            status['state'] = 'RESUME_PREPARING'
            # 경로만 계산한다. command()가 이 구간의 출력을 0으로 고정한다.
            return not resume['missing']
        decision = self.race.decide(sensors_ready=True, now_s=status['sim_time_s'], wall_s=time.monotonic())
"""),
("""                if result is not None and not self.vision_pipeline.fault:
                    self.race.observe_vision(result, self.vision_pipeline.epoch, self.now_s())
""", """                if result is not None and not self.vision_pipeline.fault:
                    self.race.observe_vision(result, self.vision_pipeline.epoch, self.now_s())
                if self.race.resume is not None:
                    now, resume = self.now_s(), self.race.resume
                    self.race.advance_resume(now, time.monotonic(), inputs=self.resume_inputs(now),
                        route=None if self.resume_route is None or
                        now-self.resume_route['stamp_s'] >= self.settings['scan_timeout_s'] else self.resume_route)
                    if self.race.resume is None and self.race.permission.latch is not None:
                        self.resume_route = None
"""),
("""                self.rgb_stamp = result['stamp']
                self.image_wall_time = result['received']
                if 30 in self.ids:
                    self.side = 'right'
                elif any(marker in self.ids for marker in (0, 20, 45)):
                    self.side = 'left'
""", """                self.rgb_stamp = result['stamp']
                self.image_wall_time = result['received']
                if getattr(self, 'race', None) is None or self.race.markers_apply():
                    if 30 in self.ids:
                        self.side, self.side_marker = 'right', 30
                    elif any(marker in self.ids for marker in (0, 20, 45)):
                        self.side = 'left'
                        self.side_marker = next(marker for marker in (0, 20, 45) if marker in self.ids)
"""),
("""            if getattr(self, 'follow_trace', None) is not None:
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
""", """            if getattr(self, 'follow_trace', None) is not None:
                self.follow_trace.record_call(self.scan, self.motion.history,
                    self.motion.last_meta.get('reference_stamp_s'), self.motion.counts['resets'],
                    speed, self.side, self.last_steering, 1/self.settings['control_rate_hz'],
                    self.motion.verified, self.rear_blind_half_angle, output)
            return self.prepared_output(output)
        return self.prepared_output(pursuit_command(points, self.side, speed,
                               self.settings["wheelbase_m"], self.settings["max_steering_angle_rad"],
                               self.settings["max_speed_mps"],
                               self.settings["lateral_acceleration_limit_mps2"],
                               self.settings["target_wall_distance_m"],
                               self.motion.last_meta.get('oldest_observation_age_s', .2)))

    def prepared_output(self, output):
        \"\"\"START 준비 중에는 계획 결과로 경로 성립만 확인하고 출력은 0으로 둔다.\"\"\"
        if getattr(self, 'race', None) is None or self.race.resume is None:
            return output
        speed, steering, detail = output
        reason = detail.get('reason')
        self.resume_route = dict(ready=bool(speed > 0 or reason in ROUTE_HOLDS), reason=reason,
                                 planned_speed_mps=float(speed), preferred_wall=self.side,
                                 path_wall=detail.get('path_wall'), stamp_s=self.now_s())
        return 0., steering, {**detail, 'reason': 'resume_preparing', 'underlying_reason': reason}
"""),
])

patch('src/arena_autonomy/arena_autonomy/lidar_safety.py', [
("""        self.last_clock = None
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive/safe", 10)
""", """        self.last_clock = None
        self.input_floor_s = -math.inf
        # 경기 운영 후보에서만 연결한다. 관측 기억을 지우기만 하므로 이 신호로 허가가 늘어나지 않는다.
        if str(self.declare_parameter('race_control_mode', 'legacy').value) == 'b3_candidate':
            self.create_subscription(String, '/race/input_boundary', self.on_input_boundary, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive/safe", 10)
"""),
("""    def on_scan(self, msg):
        if self.scan is None or seconds(msg.header.stamp) >= seconds(self.scan.header.stamp):
""", """    def on_input_boundary(self, msg):
        boundary = float(json.loads(msg.data)['boundary_s'])
        self.input_floor_s = boundary
        self.scan = None
        self.motion.begin_input_boundary(boundary)
        if getattr(self, 'partial_policy', None) is not None:
            self.partial_policy.reset()

    def on_scan(self, msg):
        if seconds(msg.header.stamp) < self.input_floor_s:
            return
        if self.scan is None or seconds(msg.header.stamp) >= seconds(self.scan.header.stamp):
"""),
("""            self.scan = self.command = None
            self.motion.history.reset()
            if getattr(self, 'partial_policy', None) is not None:
""", """            self.scan = self.command = None
            self.input_floor_s = -math.inf
            self.motion.history.reset()
            if getattr(self, 'partial_policy', None) is not None:
"""),
])
