"""r25 단독 주행 명시 STOP 시간선 분해(저장 로그 읽기 전용).

시계 영역을 섞지 않는다: gz_sim(포즈/구동 플러그인/접촉 헤더), mcu_ros_us(가상 MCU 노드 시계),
decision_ref_s(제어기 결정 입력 기준 시각). 평가용 정답 값은 원인 분리용이다.
"""
import hashlib
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
RUN = REPO / 'artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1'
CMP = REPO / 'artifacts/validation/2026-11-08/traffic_race_distance_v1/two_actor_race_distance/run1'
WINDOW = (37.3, 39.1)
INPUTS = [RUN / n for n in ('report.json', 'stop_trace.json', 'ego_poses.jsonl', 'actuation_trace.json',
                            'drivetrain_gz.jsonl', 'follow_decisions.jsonl', 'wheel_contacts.jsonl', 'trace.json')]
INPUTS += [RUN / 'runtime_vehicle.sdf']
INPUTS += [CMP / 'report.json', CMP / 'stop_trace.json']


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def r(x, n=4):
    return None if x is None else round(x, n)


def main():
    hashes = {str(p.relative_to(REPO)).replace('\\', '/'): sha256(p) for p in INPUTS}
    report = load(RUN / 'report.json')
    dense = report['dense_pose_stop_evidence']
    final_pose_s = dense['covered_last_s']
    # 평가기 stop_start는 서비스 호출 직후 최신 pose 시각. 직접 저장되지 않아 보고값으로 역산한다.
    request_s = final_pose_s - report['stop_wait_sim_s']
    stable_since_s = dense['required_first_s']

    # gz_sim: 조밀 pose로 변위 재계산
    poses = []
    with open(RUN / 'ego_poses.jsonl', encoding='utf-8') as f:
        for line in f:
            t, x, y, yaw = json.loads(line)
            if WINDOW[0] <= t <= WINDOW[1]:
                poses.append((t, x, y))
    origin = max((p for p in poses if p[0] <= request_s + 1e-9), key=lambda p: p[0])

    def disp_at(t):
        p = max((q for q in poses if q[0] <= t + 1e-9), key=lambda q: q[0])
        return math.dist(origin[1:], p[1:]), p[0]

    stop_trace = load(RUN / 'stop_trace.json')
    last_trace = load(RUN / 'trace.json')[-1]

    # gz_sim: 구동 플러그인 표본
    dt_rows = []
    with open(RUN / 'drivetrain_gz.jsonl', encoding='utf-8') as f:
        for line in f:
            d = json.loads(json.loads(line)['data'])
            if WINDOW[0] <= d['sim_time_s'] <= WINDOW[1]:
                dt_rows.append(d)
    pre = [d for d in dt_rows if d['sim_time_s'] <= request_s]
    post = [d for d in dt_rows if d['sim_time_s'] > request_s]
    applied_zero = next(d for d in post if d['applied_target_mps'] == 0)
    v_at_request = pre[-1]['truth_longitudinal_mps']
    first_body_drop = next(d for d in post if d['truth_longitudinal_mps'] < v_at_request - .02)
    first_motor_drop = next(d for d in post if d['motor_speed_rad_s'] < pre[-1]['motor_speed_rad_s'] - 5)
    peak_after = max(post, key=lambda d: d['truth_longitudinal_mps'])
    zero_cross = next(d for d in post if d['truth_longitudinal_mps'] <= 0)
    before_zero = [d for d in post if d['sim_time_s'] < zero_cross['sim_time_s']]
    negative = [d for d in post if d['truth_longitudinal_mps'] < -1e-3]
    neg_motor = [d for d in post if d['motor_speed_rad_s'] < 0]
    # 감속 구간 평균 감속도: 플러그인 목표0 이후 첫 표본~0 교차 직전 표본
    a0, a1 = applied_zero, before_zero[-1]
    mean_decel = (a0['truth_longitudinal_mps'] - a1['truth_longitudinal_mps']) / (a1['sim_time_s'] - a0['sim_time_s'])
    slip = [d for d in before_zero if d['truth_longitudinal_mps'] > .2 and d['sim_time_s'] > first_body_drop['sim_time_s']]
    ratios = [d['wheel_surface_speed_mps'] / d['truth_longitudinal_mps'] for d in slip]
    torque_lim = [d for d in post if d['limited_demand_nm'] != d['raw_demand_nm']]

    # mcu_ros_us: 가상 MCU 상태(피드백 표본은 feedback_seq 기준 중복 제거)
    act = load(RUN / 'actuation_trace.json')
    req_us = round(request_s * 1e6)
    last_active = max((s for s in act if s['state'] == 'ACTIVE'), key=lambda s: s['now_us'])
    latch = next(s for s in act if s['state'] == 'ESTOP_LATCHED')
    fb, seen = [], set()
    for s in act:
        if s['feedback_seq'] is not None and s['feedback_seq'] not in seen and \
                WINDOW[0] * 1e6 <= s['feedback_capture_us'] <= WINDOW[1] * 1e6:
            seen.add(s['feedback_seq'])
            fb.append(s)
    shaft = []
    for a, b in zip(fb, fb[1:]):
        shaft.append({'capture_us': b['feedback_capture_us'], 'counts': b['motor_counts'],
                      'delta_counts': b['motor_counts'] - a['motor_counts'],
                      'mean_wheel_mps': b['mean_wheel_mps']})
    shaft_post = [s for s in shaft if s['capture_us'] > latch['now_us']]
    shaft_neg = [s for s in shaft_post if s['delta_counts'] < 0]
    counts_peak = max(s['counts'] for s in shaft_post)
    stationary = next((s for s in act if s['state'] == 'ESTOP_LATCHED' and s['stationary_ready']), None)

    # decision_ref_s: 대회 제어기 결정 단계 출력(실제 /drive 발행값 기록은 아님)
    decisions = []
    with open(RUN / 'follow_decisions.jsonl', encoding='utf-8') as f:
        for line in f:
            o = json.loads(line)
            if o.get('kind') == 'call':
                t = o['data']['input'].get('reference_s')
                if t is not None and WINDOW[0] <= t <= WINDOW[1]:
                    decisions.append((t, o['data']['output'][0]))
    dec_post = [d for d in decisions if d[0] > request_s]

    # gz_sim: 정지 창 바퀴 접촉. 헤더 시각만 정규식으로 보고 창 안만 해석한다.
    # protobuf JSON은 int64 sec를 문자열로 쓴다("sec":"37"). 0초면 sec가 생략된다.
    skid_span = (first_body_drop['sim_time_s'] + .05, before_zero[-1]['sim_time_s'] - .05)
    stamp_re = re.compile(r'"stamp":\{(?:"sec":"?(\d+)"?,?)?(?:"nsec":(\d+))?\}')
    contacts = {}
    lo, hi = request_s - .1, final_pose_s
    with open(RUN / 'wheel_contacts.jsonl', encoding='utf-8') as f:
        for line in f:
            m = stamp_re.search(line, 0, 200)
            if not m:
                continue
            t = int(m.group(1) or 0) + int(m.group(2) or 0) * 1e-9
            if t < lo:
                continue
            if t > hi:
                break
            for c in json.loads(line).get('contact', []):
                name = c['collision1']['name'].split('::')[1]
                force = [sum(w['body1Wrench']['force'].get(k, 0.) for w in c.get('wrench', [])) for k in 'xyz']
                row = contacts.setdefault(name, {}).setdefault(round(t, 3), [0., 0., 0.])
                for i in range(3):
                    row[i] += force[i]
    all_stamps = sorted({t for v in contacts.values() for t in v})
    wheel_contact = {}
    for name, rows in sorted(contacts.items()):
        missing = [t for t in all_stamps if t not in rows]
        # raw-component diagnostic: body1Wrench.force의 z를 수직, xy를 접선으로 단순 취급한다.
        # 보고 좌표·collision1/2 순서·normal 방향을 확인하지 않았으므로 검증된 접선/수직력이 아니다.
        normal = [v[2] for v in rows.values()]
        skid = [math.hypot(v[0], v[1]) / v[2] for t, v in rows.items()
                if skid_span[0] <= t <= skid_span[1] and v[2] > 0]
        skid_n = [v[2] for t, v in rows.items() if skid_span[0] <= t <= skid_span[1]]
        wheel_contact[name] = {'stamps_with_contact': len(rows), 'stamps_total': len(all_stamps),
                               'stamps_missing': len(missing), 'first_missing_s': missing[0] if missing else None,
                               'raw_z_min_n': r(min(normal), 3), 'raw_z_max_n': r(max(normal), 3),
                               'skid_raw_z_mean_n': r(sum(skid_n) / len(skid_n), 3),
                               'skid_raw_xy_over_z_mean': r(sum(skid) / len(skid), 3),
                               'skid_raw_xy_over_z_max': r(max(skid), 3)}

    sdf = (RUN / 'runtime_vehicle.sdf').read_text(encoding='utf-8')

    def tag(name):
        return float(re.search(f'<{name}>([^<]+)</{name}>', sdf).group(1))
    gear, radius, mass = tag('gear_ratio'), tag('wheel_radius'), report['simulated_total_mass_kg']
    skid_rows = [d for d in post if skid_span[0] <= d['sim_time_s'] <= skid_span[1]]
    mean_torque = sum(d['motor_torque_nm'] for d in skid_rows) / len(skid_rows)
    rear_n = sum(wheel_contact[w]['skid_raw_z_mean_n'] for w in ('rear_left_wheel', 'rear_right_wheel'))
    force_balance = {
        'skid_span_gz_sim_s': [r(skid_span[0], 3), r(skid_span[1], 3)],
        'model_values_runtime_vehicle_sdf': {'gear_ratio': r(gear, 6), 'wheel_radius_m': radius,
                                             'motor_brake_torque_limit_nm': tag('motor_brake_torque_limit'),
                                             'driven_wheels': 'rear_left/right (단일 모터)',
                                             'first_mu_tag_in_vehicle_sdf': tag('mu'),
                                             'total_mass_kg': mass},
        'mu_boundary': ('first_mu_tag_in_vehicle_sdf는 차량 SDF 전체의 첫 <mu> 값일 뿐이다. 노면 SDF의 <mu>0.8과 결합 규칙·'
                        '물리 엔진 마찰 근사를 확인하지 않았으므로 바퀴-노면 유효 마찰계수로 쓰지 않는다.'),
        'body_decel_force_n': r(mass * mean_decel, 3),
        'mean_motor_torque_nm': r(mean_torque, 4),
        'motor_torque_as_wheel_ground_force_n': r(abs(mean_torque) * gear / radius, 3),
        'brake_torque_limit_as_ground_force_n': r(tag('motor_brake_torque_limit') * gear / radius, 3),
        'rear_raw_z_sum_n_diagnostic': r(rear_n, 3),
        'note': ('모터 토크×기어비/바퀴 반경 환산은 바퀴 각가속도·관성·구름저항·기어/접촉 손실을 무시한 근사다. '
                 '차체 감속력과 크기가 비슷하다는 대조일 뿐이며 마찰 포화의 증명이 아니다. 시뮬 모델값이며 실차 값이 아니다.'),
    }

    # 비교 1건(다른 평가기): 요청 시점 속도와 변위만
    cr = load(CMP / 'report.json')
    cst = load(CMP / 'stop_trace.json')
    comparison = {
        'run': str(CMP.relative_to(REPO)).replace('\\', '/'),
        'evaluator': 'g1_traffic_eval (다른 평가기, 단계 분해 불가: drivetrain_gz 미기록)',
        'stop_request_sim_s': cr['stop_request_sim_s'],
        'ground_speed_first_stop_trace_mps': r(cst[0]['ground_speed_mps']),
        'ground_speed_source_s': cst[0].get('dynamics_source_s'),
        'note': '첫 stop_trace 속도의 출처 시각이 요청보다 0.15초 앞선다. 요청 시점 실제 속도로 쓰지 않는다.',
        'max_ground_speed_in_stop_trace_mps': r(max(s['ground_speed_mps'] for s in cst)),
        'displacement_m': r(cr['stop']['displacement_m']),
        'stable_since_s': cr['stop']['pose_evidence']['required_first_s'],
        'stable_since_minus_request_s': r(cr['stop']['pose_evidence']['required_first_s'] - cr['stop_request_sim_s'], 3),
    }

    def dtrow(d):
        return {'sim_time_s': d['sim_time_s'], 'applied_target_mps': r(d['applied_target_mps']),
                'motor_rad_s': r(d['motor_speed_rad_s'], 2), 'motor_torque_nm': r(d['motor_torque_nm']),
                'left_wheel_rad_s': r(d['left_speed_rad_s'], 3), 'right_wheel_rad_s': r(d['right_speed_rad_s'], 3),
                'wheel_surface_mps': r(d['wheel_surface_speed_mps']), 'truth_longitudinal_mps': r(d['truth_longitudinal_mps']),
                'watchdog_stop': d['watchdog_stop']}

    disp_zero, _ = disp_at(zero_cross['sim_time_s'])
    disp_stable, _ = disp_at(stable_since_s)
    disp_final, final_t = disp_at(final_pose_s)
    events = [
        {'event': '제어기 마지막 기록 trace(평가 루프 종료 전)', 'gz_sim_s': last_trace['time'],
         'decision_ref_s': last_trace['autonomy']['sim_time_s'],
         'value': {'speed_command_mps': r(last_trace['autonomy']['speed_command_mps']),
                   'independent_safe_speed_mps': r(last_trace.get('safe_drive_speed_mps'))}},
        {'event': '평가기 STOP 요청(서비스 호출 시 최신 pose, 역산)', 'gz_sim_s': r(request_s, 3),
         'basis': 'report.dense_pose_stop_evidence.covered_last_s - report.stop_wait_sim_s',
         'bracket_gz_sim_s': [last_trace['time'], stop_trace[0]['time']], 'eval_wall_s': '미기록'},
        {'event': 'MCU 마지막 ACTIVE 상태', 'mcu_ros_us': last_active['now_us'],
         'value': {'target_speed_mps': r(last_active['target_speed_mps']), 'accepted_seq': last_active['accepted_seq']}},
        {'event': 'MCU STOP 수신·래치(첫 ESTOP_LATCHED 상태)', 'mcu_ros_us': latch['now_us'],
         'value': {'reason': latch['reason'], 'target_speed_mps': latch['target_speed_mps'],
                   'output_ready': latch['output_ready'], 'motor_counts': latch['motor_counts']},
         'note': 'mcu_ros_us - 요청(gz_sim) 차이는 같은 /clock 유도 가정의 참고값',
         'reference_minus_request_ms_assumed_same_clock': r((latch['now_us'] - req_us) / 1e3, 1)},
        {'event': '구동 플러그인 적용 목표 0(첫 표본)', 'gz_sim_s': applied_zero['sim_time_s'],
         'bracket_gz_sim_s': [pre[-1]['sim_time_s'], applied_zero['sim_time_s']], 'value': dtrow(applied_zero),
         'displacement_m': r(disp_at(applied_zero['sim_time_s'])[0])},
        {'event': '모터축 각속도 첫 감소(>5 rad/s)', 'gz_sim_s': first_motor_drop['sim_time_s'], 'value': dtrow(first_motor_drop)},
        {'event': '차체속도 첫 감소(>0.02 m/s, 평가 전용)', 'gz_sim_s': first_body_drop['sim_time_s'], 'value': dtrow(first_body_drop),
         'displacement_m': r(disp_at(first_body_drop['sim_time_s'])[0])},
        {'event': '차체 전진속도 0 교차(평가 전용)', 'gz_sim_s': zero_cross['sim_time_s'], 'value': dtrow(zero_cross),
         'displacement_m': r(disp_zero)},
        {'event': '차체 후진 최대(평가 전용)', 'gz_sim_s': min(negative, key=lambda d: d['truth_longitudinal_mps'])['sim_time_s'],
         'value': dtrow(min(negative, key=lambda d: d['truth_longitudinal_mps']))},
        {'event': 'MCU 모터축 카운트 감소(음의 축 회전) 첫 표본', 'mcu_ros_us': shaft_neg[0]['capture_us'] if shaft_neg else None,
         'value': shaft_neg[0] if shaft_neg else None},
        {'event': '평가기 안정 시작(odom·차체<0.01 m/s 유지 시작)', 'gz_sim_s': stable_since_s, 'displacement_m': r(disp_stable)},
        {'event': 'MCU stationary_ready 첫 True', 'mcu_ros_us': stationary['now_us'] if stationary else None},
        {'event': '평가기 0.5s 안정 창 종료·판정', 'gz_sim_s': final_pose_s, 'displacement_m': r(disp_final),
         'report_displacement_m': r(report['stop_displacement_m'])},
    ]
    phases_gz = {
        'request_to_plugin_zero_upper_s': r(applied_zero['sim_time_s'] - request_s, 3),
        'request_to_first_body_drop_upper_s': r(first_body_drop['sim_time_s'] - request_s, 3),
        'request_to_zero_cross_s': r(zero_cross['sim_time_s'] - request_s, 3),
        'zero_cross_to_stable_since_s': r(stable_since_s - zero_cross['sim_time_s'], 3),
        'stable_window_s': r(final_pose_s - stable_since_s, 3),
        'total_stop_wait_s': r(final_pose_s - request_s, 3),
    }
    summary = {
        'scope': 'r25 단독 주행 명시 STOP 저장 로그 분해. 새 실행 없음. 시뮬 결과이며 실제 회로/실차 제동 증명 아님.',
        'run': str(RUN.relative_to(REPO)).replace('\\', '/'),
        'stop_mode': report['actuation']['stop_mode'],
        'clock_domains': {
            'gz_sim_s': 'Gazebo 시뮬 시각(pose 헤더·drivetrain sim_time_s·접촉 헤더). 구간 계산은 이 영역 안에서만 한다.',
            'mcu_ros_us': '가상 MCU 노드 ROS 시계 µs. /clock 유도로 보이나 gz_sim과 동일 시계임은 별도 확인하지 않았다.',
            'decision_ref_s': '제어기 결정 입력 reference_s(센서 기준). 평가기 STOP과 같은 축으로 빼지 않는다.',
            'eval_wall_s': 'STOP 구간 미기록.',
        },
        'events': events,
        'phases_gz_sim': phases_gz,
        'deceleration': {
            'speed_at_request_mps': r(v_at_request), 'speed_at_request_sample_s': pre[-1]['sim_time_s'],
            'peak_after_request_mps': r(peak_after['truth_longitudinal_mps']), 'peak_after_request_s': peak_after['sim_time_s'],
            'mean_decel_mps2': r(mean_decel, 3),
            'mean_decel_span_s': [a0['sim_time_s'], a1['sim_time_s']],
            'kinematic_distance_v2_over_2a_m': r(v_at_request ** 2 / (2 * mean_decel), 3),
            'torque_limited_samples_after_request': len(torque_lim),
            'limited_demand_min_nm': r(min(d['limited_demand_nm'] for d in post)),
            'slip_wheel_surface_over_body': {
                'samples': len(slip), 'min': r(min(ratios), 3), 'median': r(sorted(ratios)[len(ratios) // 2], 3),
                'max': r(max(ratios), 3),
                'condition': '첫 차체감속 이후, 0 교차 이전, 차체>0.2 m/s',
                'negative_motor_sample_times_s': [d['sim_time_s'] for d in slip if d['motor_speed_rad_s'] < 0],
                'note': '약21ms 표본 기준. 표본 사이 순간 잠김 여부는 알 수 없으며 지속 잠김은 표본에서 확인되지 않았다는 뜻으로만 쓴다.',
            },
            'left_right_wheel_rad_s_max_abs_diff_after_request': r(max(abs(d['left_speed_rad_s'] - d['right_speed_rad_s']) for d in post), 3),
            'negative_body_samples': len(negative),
            'min_body_mps': r(min(d['truth_longitudinal_mps'] for d in post)),
            'negative_motor_samples': len(neg_motor),
            'min_motor_rad_s': r(min(d['motor_speed_rad_s'] for d in post), 2),
            'watchdog_stop_any_after_request': any(d['watchdog_stop'] for d in post),
        },
        'mcu_shaft_encoder': {
            'feedback_samples_after_latch': len(shaft_post),
            'counts_at_latch': latch['motor_counts'], 'counts_peak': counts_peak,
            'counts_final': shaft_post[-1]['counts'],
            'net_counts_after_peak': shaft_post[-1]['counts'] - counts_peak,
            'negative_delta_samples': len(shaft_neg),
            'negative_delta_counts_total': sum(s['delta_counts'] for s in shaft_neg),
            'm_per_count_from_contract': r(math.tau * .0325 / (28 * 8.9706), 7),
        },
        'controller_decisions_after_request': {
            'count': len(dec_post),
            'speed_command_min_mps': r(min(d[1] for d in dec_post)) if dec_post else None,
            'speed_command_max_mps': r(max(d[1] for d in dec_post)) if dec_post else None,
            'first_ref_s': dec_post[0][0] if dec_post else None, 'last_ref_s': dec_post[-1][0] if dec_post else None,
            'note': 'software_stop는 MCU 래치로 정지시키며 대회 제어기 명령은 0으로 바뀌지 않았다. 결정 단계 출력이며 /drive 발행값 자체는 아니다.',
        },
        'interpretation': {
            'confirmed_from_logs': [
                'STOP 시간선과 구간 분해(gz_sim 영역): 요청→플러그인 목표0 ≤0.010s, 첫 차체감속 ≤0.030s, 0 교차 0.763s, 안정 시작까지 0.183s, 판정 창 0.5s',
                '플러그인 목표0 이후 모터축·후륜 각속도 급감, 후륜 표면속도가 차체속도보다 크게 낮음(큰 종방향 슬립)',
                '정지 창 네 바퀴 모두 매 접촉 시각 접촉 메시지 존재',
                '음의 모터 회전 표본과 차체 역주행 표본, MCU 모터축 카운트 감소',
                '대회 제어기 결정 단계는 요청 뒤에도 양의 속도 명령, 정지는 MCU software_stop 래치',
            ],
            'unconfirmed': [
                '마찰 포화가 감속(3.285 m/s²)의 지배 원인인지: 접촉 wrench 보고 좌표·collision 순서·normal 방향과 유효 마찰계수를 확인하지 않았다',
                '원시 xy/z 비 약1.10은 raw-component diagnostic이며 검증된 접선/수직력 비가 아니다',
                '표본 사이 순간 잠김 여부',
            ],
        },
        'force_balance_skid': force_balance,
        'wheel_contacts_stop_window': {'window_gz_sim_s': [r(lo, 3), hi], 'by_wheel': wheel_contact},
        'not_recorded': [
            '평가기 STOP 서비스 호출 wall 시각/ROS 시각(보고값에서 역산만 가능)',
            'STOP 이후 /drive(대회 명령) 실제 발행값 — 결정 단계 출력만 있음',
            'STOP 이후 /drive/safe(독립 보호 출력) — trace.json은 37.523s에서 끝남',
            '/actuation/output_status 시계열 — 최종값만 report에 있음',
            '구동 싱크(guarded_sim_actuator)가 플러그인에 0을 쓴 시각',
            '좌우 바퀴 개별 엔코더(MCU 모드는 모터축 단일 채널)',
            '구동 플러그인 매 스텝 값(로그는 약21ms 간격 표본)',
        ],
        'stop_trace_check': {
            'first_s': stop_trace[0]['time'], 'last_s': stop_trace[-1]['time'], 'samples': len(stop_trace),
            'collision_samples': sum(s['collision'] for s in stop_trace),
            'origin_pose_s': origin[0],
        },
        'comparison_one_run': comparison,
    }
    out = HERE / 'summary.json'
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
    (HERE / 'input_sha256.json').write_text(json.dumps(hashes, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({'phases_gz_sim': phases_gz, 'decel': summary['deceleration']['mean_decel_mps2'],
                      'disp_final': events[-1]['displacement_m'], 'report_disp': events[-1]['report_displacement_m']},
                     ensure_ascii=False))


if __name__ == '__main__':
    sys.exit(main())
