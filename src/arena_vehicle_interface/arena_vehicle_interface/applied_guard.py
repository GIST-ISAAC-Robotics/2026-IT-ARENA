"""가상 MCU 뒤 Gazebo 전용 수신 감시. 실제 ESC watchdog을 대체하지 않는다."""
from arena_vehicle_interface.actuation_contract import integer, number


class AppliedGuard:
    def __init__(self, *, timeout_us=100_000, wall_timeout_s=3., max_speed=2.5, max_angle=.45):
        if not integer(timeout_us, 1) or not number(wall_timeout_s) or wall_timeout_s <= 0:
            raise ValueError('invalid timeout')
        if not number(max_speed) or max_speed <= 0 or not number(max_angle) or max_angle <= 0:
            raise ValueError('invalid limits')
        self.timeout_us, self.wall_timeout_s = timeout_us, wall_timeout_s
        self.max_speed, self.max_angle = max_speed, max_angle
        self.epoch = None
        self.seen_boots = set()
        self.blocked = True
        self.last_stamp = self.last_wall = self.last_now = None
        self.deadline_us = None
        self.speed = self.steering = 0.
        self.reason = 'awaiting_explicit_arm'

    def stop(self, reason):
        self.speed, self.blocked, self.reason = 0., True, reason

    def update(self, status, now_us, wall_s):
        if not integer(now_us) or not number(wall_s):
            self.stop('invalid_clock')
            return False
        self.watch(now_us, wall_s)
        try:
            stamp, speed, steering = status['now_us'], status['target_speed_mps'], status['steering_rad']
            epoch = (status['boot'], status['generation'])
            valid = (type(epoch[0]) is str and 1 <= len(epoch[0]) <= 64 and integer(epoch[1]) and
                     integer(stamp) and 0 <= now_us - stamp < self.timeout_us and
                     number(speed) and 0 <= speed <= self.max_speed and
                     number(steering) and abs(steering) <= self.max_angle and
                     (status['state'] != 'ARMED' or speed == 0.))
            if not valid:
                self.stop('invalid_applied_status')
                return False
            if epoch != self.epoch:
                if self.epoch is not None and ((epoch[0] == self.epoch[0] and epoch[1] < self.epoch[1]) or
                                               (epoch[0] != self.epoch[0] and epoch[0] in self.seen_boots)):
                    return False
                self.seen_boots.add(epoch[0])
                self.epoch, self.last_stamp = epoch, None
                self.blocked = True
                # 새 허가를 실제로 관측해야 한다. ACTIVE만 재등장하면 재개하지 않는다.
                if status['state'] == 'ARMED' and speed == 0.:
                    self.blocked = False
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return False
            self.last_stamp, self.last_wall = stamp, wall_s
            self.steering = steering
            if status['state'] not in ('ARMED', 'ACTIVE'):
                self.stop(status['reason'])
            elif not integer(status['deadline_us']) or now_us >= status['deadline_us']:
                self.stop('applied_command_expired')
            elif not self.blocked:
                self.speed, self.reason = speed, 'accepted'
                self.deadline_us = status['deadline_us']
            return not self.blocked
        except (KeyError, TypeError):
            self.stop('malformed_applied_status')
            return False

    def watch(self, now_us, wall_s):
        if not integer(now_us) or not number(wall_s):
            self.stop('invalid_clock')
            return
        if self.last_now is not None and now_us < self.last_now:
            self.stop('clock_regressed')
        self.last_now = now_us
        if not self.blocked and self.deadline_us is not None and now_us >= self.deadline_us:
            self.stop('applied_command_expired')
        if self.last_stamp is not None and now_us - self.last_stamp >= self.timeout_us:
            self.stop('applied_stream_timeout')
        if self.last_wall is not None and wall_s - self.last_wall >= self.wall_timeout_s:
            self.stop('applied_wall_timeout')
