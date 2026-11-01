"""before.zip과 같은 바이트 위에 합의된 후보 A만 적용한다. 다른 변경이 있으면 멈춘다."""
import hashlib
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
FOLLOW = 'src/arena_autonomy/arena_autonomy/local_follow.py'
PURSUIT = 'src/arena_autonomy/arena_autonomy/local_pursuit.py'
LAUNCH = 'src/arena_bringup/launch/simulation.launch.py'
BASIC = 'scripts/validate_basic_autonomy.py'

HELPER = '''def path_agrees_arc(candidate, reference, tolerance, lo=.05, hi=1.):
    """차체 x 창이 비는 급커브에서 두 경로의 같은 호길이 구간을 같은 기준으로 비교한다.

    path_agrees와 마찬가지로 공통 구간 0.25 m·각 5점을 요구하며 외삽하지 않는다.
    벽 정체를 인증하지 않고, 공통 80%를 넘긴 뒤 갈라지는 먼 분기는 구별하지 못한다.
    """
    def ahead(path):
        path = np.asarray(path, dtype=float)
        path = path[int(np.argmin(np.linalg.norm(path, axis=1))):]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        keep = (arc >= lo) & (arc <= hi)
        return path[keep], arc[keep]
    (first, first_arc), (second, second_arc) = ahead(candidate), ahead(reference)
    if len(first) < 5 or len(second) < 5:
        return False
    start, end = max(first_arc[0], second_arc[0]), min(first_arc[-1], second_arc[-1])
    if end-start < .25:
        return False
    first = first[(first_arc >= start-1e-12) & (first_arc <= end+1e-12)]
    second = second[(second_arc >= start-1e-12) & (second_arc <= end+1e-12)]
    if len(first) < 5 or len(second) < 5:
        return False
    distances = np.linalg.norm(first[:, None, :]-second[None, :, :], axis=2)
    return bool(max(np.percentile(np.min(distances, axis=0), 80),
                    np.percentile(np.min(distances, axis=1), 80)) <= tolerance)


'''
BRANCH = '''                    if (not eligible and cfg.low_speed_arc_reacquire and abs(speed) <= .10
                            and sufficient(opposite, other)
                            and path_agrees_arc(candidate, opposite, cfg.boundary_agreement_m)):
                        # 측정 평균 속도가 낮을 때만 쓴다. 단일 축이라 실제 정지를 증명하지 않으며
                        # 이후 확인 횟수·점유·재개 여유·독립 보호는 기존 판정을 그대로 따른다.
                        eligible = True
                        detail['bilateral_reference'] = 'low_speed_arc'
'''
STEER = '''                if cfg.low_speed_arc_reacquire and abs(speed) <= .10:
                    # 속도 0을 유지한 채 조향만 경로 쪽으로 돌린다. 전진은 재개 여유와 독립 보호가 정한다.
                    # 스캔 말미의 시간 여유 증가로 생긴 낮은 값이 새 스캔의 재개 계수를 지우지 않게 한다.
                    self.held = True
                    return 0., steering, {**detail, 'reason': 'follow_steer_in_place', 'behavior': 'HOLD'}
'''
FIELD = '    preview_turn_speed: bool = False\n'
EXEMPT = "'preview_turn_speed', 'boundary_evidence_mode',"
PREVIEW_ERROR = "            raise ValueError('turn preview requires observed route speed and timed beam occupancy')\n"
SUPPORT = 'def boundary_support(points, path, side, config):\n'
AGREE = '                    eligible = sufficient(opposite, other) and path_agrees(candidate, opposite, cfg.boundary_agreement_m)\n'
GAP = "                return self._hold('follow_gap_hold', detail)\n"
PARAM = "            preview_turn_speed=bool(self.declare_parameter('g1_preview_turn_speed',False).value),\n"
PROFILE = '''SPEED_PROFILES["g1_open_10kmh_turn15"] = {
    **SPEED_PROFILES["g1_open_10kmh_rise15"], "g1_preview_turn_speed": True}
'''
EDITS = {
    FOLLOW: [
        (FIELD, FIELD+'    low_speed_arc_reacquire: bool = False  # 단독 시험 선택. 낮은 측정 속도의 양측 경로 재획득만.\n'),
        (EXEMPT, "'preview_turn_speed', 'low_speed_arc_reacquire', 'boundary_evidence_mode',"),
        (PREVIEW_ERROR, PREVIEW_ERROR+
         "        if (type(self.low_speed_arc_reacquire) is not bool or\n"
         "                (self.low_speed_arc_reacquire and not self.blend_open_speed_by_length)):\n"
         "            raise ValueError('low speed arc reacquire requires the solo observed route speed profile')\n"),
        (SUPPORT, HELPER+SUPPORT), (AGREE, AGREE+BRANCH), (GAP, STEER+GAP)],
    PURSUIT: [(PARAM, PARAM+"            low_speed_arc_reacquire=bool(self.declare_parameter('g1_low_speed_arc_reacquire',False).value),\n")],
    LAUNCH: [(PROFILE, PROFILE+
              '# 위 프로필에 낮은 측정 속도 재획득 선택만 더한 단독 시험. 교통 구성에는 연결하지 않는다.\n'
              'SPEED_PROFILES["g1_open_10kmh_turn15_arc"] = {\n'
              '    **SPEED_PROFILES["g1_open_10kmh_turn15"], "g1_low_speed_arc_reacquire": True}\n')],
    BASIC: [
        ('"g1_open_10kmh_rise15", "g1_open_10kmh_turn15", "hardware_target"',
         '"g1_open_10kmh_rise15", "g1_open_10kmh_turn15", "g1_open_10kmh_turn15_arc", "hardware_target"'),
        ("and args.speed_profile in ('g1_open_2p0_approach','g1_open_2p0_turn','g1_open_10kmh_rise15','g1_open_10kmh_turn15')",
         "and args.speed_profile in ('g1_open_2p0_approach','g1_open_2p0_turn','g1_open_10kmh_rise15','g1_open_10kmh_turn15','g1_open_10kmh_turn15_arc')"),
        ("if args.speed_profile in ('g1_open_2p0_turn','g1_open_10kmh_turn15'):",
         "if args.speed_profile in ('g1_open_2p0_turn','g1_open_10kmh_turn15','g1_open_10kmh_turn15_arc'):"),
        ('def g1_solo_arguments(enabled, boundary, occupancy, record, output, corridor=False, recent=False):\n',
         'def imu_sample(message):\n'
         '    """평가 진단 전용 최근 IMU 표본. 제어·보호 입력에는 쓰지 않는다."""\n'
         '    if message is None:\n'
         '        return None\n'
         '    linear, angular = message.linear_acceleration, message.angular_velocity\n'
         '    return {"stamp_s": float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9,\n'
         '            "linear_acceleration_mps2": [linear.x, linear.y, linear.z],\n'
         '            "angular_velocity_rad_s": [angular.x, angular.y, angular.z]}\n\n\n'
         'def g1_solo_arguments(enabled, boundary, occupancy, record, output, corridor=False, recent=False):\n'),
        ('"dynamics": state["dynamics"], "safety": state.get("safety"),\n',
         '"dynamics": state["dynamics"], "safety": state.get("safety"), "imu": imu_sample(state.get("imu")),\n')],
}


def main():
    with zipfile.ZipFile(HERE/'before.zip') as archive:
        before = {path: archive.read(path) for path in EDITS}
    updated = {}
    for path, edits in EDITS.items():
        raw = (ROOT/path).read_bytes()
        if raw != before[path]:
            sys.exit(f'수정 전 바이트와 다름: {path}')
        crlf = b'\r\n' in raw
        text = raw.decode('utf-8').replace('\r\n', '\n')
        for old, new in edits:
            if text.count(old) != 1:
                sys.exit(f'고유 위치를 찾지 못함: {path}: {old[:60]}')
            text = text.replace(old, new)
        updated[path] = (text.replace('\n', '\r\n') if crlf else text).encode('utf-8')
    for path, data in updated.items():
        (ROOT/path).write_bytes(data)
        print(path, hashlib.sha256(data).hexdigest(), 'crlf' if b'\r\n' in data else 'lf')


if __name__ == '__main__':
    main()
