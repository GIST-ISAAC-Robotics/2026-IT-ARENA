"""65 mm 바퀴에서 전체 감속비 15:1(초기 구성)과 비교안의 근사 계산.

저장소 루트에서 `python3 artifacts/validation/2026-10-02/gearing_65mm/compute_gearing.py`로 실행한다.
표준 라이브러리만 사용하며 ROS·센서·모터에 연결하지 않는다. 실측이 아니라 판매처 사양 두 점으로 만든
선형 브러시 DC 모델과 가정값의 계산이다. 결과는 같은 폴더의 `results.json`에 쓴다.

PR #4 원본 계산(docs/archive/2026-10-01-drivetrain/)과의 차이:
- 고정 PWM 속도 저하율을 `저하 rpm / 부하 전 무부하 rpm`으로 정의한다. 1 이상이면 그 PWM에서
  해당 부하를 이기지 못해 정지한다는 뜻으로 따로 표시한다.
- 단락 제동 전류를 드라이버 한도로 자르지 않는다. 역기전력/권선 저항의 초기 근사이며
  인덕턴스·드라이버 동작·배선 저항으로 실제 값은 달라진다.
- 차량 부하는 접지 한계 가속력과 순항 저항 가정값을 나누어 최고속도와 필요 PWM에 반영한다.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

G = 9.80665
OUT = Path(__file__).with_name("results.json")

# 판매처 상세 사양 판독값(24 V 기준). 실측 아님.
MOTOR = {"voltage_v": 24.0, "no_load_rpm": 18000.0, "no_load_a": 2.0,
         "rated_rpm": 16474.0, "rated_a": 10.28, "rated_torque_kgfcm": 1.079}

INPUTS = {
    "wheel_outer_diameter_m": 0.065,  # 2026-10-02 하드웨어팀 확정(사용자 전달)
    "ratios": {
        "15": {"stages": "피니언/스퍼 6:1 × 차동 2.5:1", "status": "하드웨어팀 초기 구성(순정 기어)"},
        "12": {"stages": "피니언/스퍼 4.8:1 × 차동 2.5:1", "status": "비교안. 부품 미확인"},
        "2.5": {"stages": "앞단 없이 차동 2.5:1", "status": "10/1 전달값. 참고 비교"},
    },
    # 이하 가정값
    "mass_kg": 2.0,
    "rear_weight_fraction": 0.5,
    "mu": 1.0,
    "gear_efficiency": 0.85,
    "cruise_resistance_n": 1.0,  # 구름·구동계 외 저항 가정. 실측 아님
    "battery_v": {"full_6s": 25.2, "nominal_6s": 22.2, "low": 21.0, "near_cutoff": 20.0},
    "encoder_ppr_candidates": [7, 13],  # PGE-213 7/13 표기 불일치
    "quadrature_factor": 4,
    "control_period_s": 0.02,
    "check_speeds_kmh": [9.0, 10.0],  # 9 km/h 명령 상한, 10 km/h 사용자 기대치
    "footprint_m": {"length": 0.20, "width": 0.15},
    "provisional_wheelbase_m": 0.145,
    "provisional_track_width_m": 0.135,
}


def rad_s(rpm: float) -> float:
    return rpm * 2 * math.pi / 60


def motor_model() -> dict:
    """V = I·R + Ke·ω, T = Kt·(I − I0)을 무부하점·정격점으로 푼다."""
    w0, wr, v = rad_s(MOTOR["no_load_rpm"]), rad_s(MOTOR["rated_rpm"]), MOTOR["voltage_v"]
    r = v * (w0 - wr) / (MOTOR["rated_a"] * w0 - MOTOR["no_load_a"] * wr)
    ke = (v - MOTOR["no_load_a"] * r) / w0
    kt = MOTOR["rated_torque_kgfcm"] * G / 100 / (MOTOR["rated_a"] - MOTOR["no_load_a"])
    return {"resistance_ohm": r, "ke_v_s_per_rad": ke, "kt_nm_per_a": kt, "i0_a": MOTOR["no_load_a"]}


M = motor_model()
D = INPUTS["wheel_outer_diameter_m"]
RADIUS = D / 2
ETA = INPUTS["gear_efficiency"]
TRACTION_N = INPUTS["mu"] * INPUTS["rear_weight_fraction"] * INPUTS["mass_kg"] * G


def motor_rpm_at(kmh: float, n: float) -> float:
    return kmh / 3.6 / (math.pi * D) * 60 * n


def kmh_at(rpm: float, n: float) -> float:
    return rpm / n / 60 * math.pi * D * 3.6


def motor_current(wheel_force_n: float, n: float) -> float:
    return M["i0_a"] + wheel_force_n * RADIUS / (n * ETA) / M["kt_nm_per_a"]


def top_speed_kmh(n: float, volts: float, wheel_force_n: float) -> float:
    """PWM 100%에서 바퀴 저항력과 균형을 이루는 속도."""
    w = (volts - motor_current(wheel_force_n, n) * M["resistance_ohm"]) / M["ke_v_s_per_rad"]
    return max(0.0, kmh_at(w * 60 / (2 * math.pi), n))


def required_duty(kmh: float, n: float, volts: float, wheel_force_n: float) -> float:
    """주어진 속도와 바퀴 저항력을 유지하는 평균 PWM duty(이상적 평균 전압 근사)."""
    w = rad_s(motor_rpm_at(kmh, n))
    return (motor_current(wheel_force_n, n) * M["resistance_ohm"] + M["ke_v_s_per_rad"] * w) / volts


def fixed_pwm_drop(kmh: float, n: float, wheel_force_n: float) -> dict:
    """무부하에서 kmh가 되는 PWM을 고정하고 부하를 걸었을 때의 회전수 저하."""
    no_load_rpm = motor_rpm_at(kmh, n)
    extra_torque = wheel_force_n * RADIUS / (n * ETA)
    drop_rpm = extra_torque / M["kt_nm_per_a"] * M["resistance_ohm"] / M["ke_v_s_per_rad"] * 60 / (2 * math.pi)
    fraction = drop_rpm / no_load_rpm
    return {"no_load_rpm": no_load_rpm, "drop_rpm": drop_rpm, "drop_fraction": fraction,
            "stalls": fraction >= 1.0}


def evaluate(n: float) -> dict:
    nominal = INPUTS["battery_v"]["nominal_6s"]
    forces = {"no_load": 0.0, "cruise_assumed": INPUTS["cruise_resistance_n"], "traction_limit": TRACTION_N}
    top = {name: {f: round(top_speed_kmh(n, v, force), 2) for f, force in forces.items()}
           for name, v in INPUTS["battery_v"].items()}
    speed_checks = {}
    for kmh in INPUTS["check_speeds_kmh"]:
        rpm = motor_rpm_at(kmh, n)
        e_back = M["ke_v_s_per_rad"] * rad_s(rpm)
        short_a = e_back / M["resistance_ohm"]
        # 전자기 토크 Kt·I만 반영한다. 모터·기어 마찰은 제동을 더 키우는 쪽이라 빼 두는 편이 보수적이다.
        short_force = M["kt_nm_per_a"] * short_a * n * ETA / RADIUS
        speed_checks[f"{kmh:g}"] = {
            "motor_rpm": round(rpm),
            "duty_no_load": {k: round(required_duty(kmh, n, v, 0.0), 4) for k, v in INPUTS["battery_v"].items()},
            "duty_traction_limit": {k: round(required_duty(kmh, n, v, TRACTION_N), 4)
                                    for k, v in INPUTS["battery_v"].items()},
            "fixed_pwm_drop_traction_limit": fixed_pwm_drop(kmh, n, TRACTION_N),
            "short_brake_initial_a_uncapped": round(short_a, 1),
            "short_brake_wheel_force_n": round(short_force, 1),
            "short_brake_over_traction_limit": round(short_force / TRACTION_N, 1),
        }
    encoder = {}
    for ppr in INPUTS["encoder_ppr_candidates"]:
        counts_rev = ppr * INPUTS["quadrature_factor"] * n
        counts_m = counts_rev / (math.pi * D)
        v9 = 9 / 3.6
        max_rpm = MOTOR["no_load_rpm"] * INPUTS["battery_v"]["full_6s"] / MOTOR["voltage_v"]
        encoder[str(ppr)] = {
            "counts_per_wheel_rev": round(counts_rev, 1),
            "mm_per_count": round(1000 / counts_m, 3),
            "counts_per_period_at_9kmh": round(v9 * counts_m * INPUTS["control_period_s"], 1),
            "speed_quantum_per_period_mps": round(1 / counts_m / INPUTS["control_period_s"], 4),
            "edge_rate_at_9kmh_hz": round(motor_rpm_at(9, n) / 60 * ppr * INPUTS["quadrature_factor"]),
            "edge_rate_at_full_battery_no_load_hz": round(max_rpm / 60 * ppr * INPUTS["quadrature_factor"]),
        }
    no_load_top = top["nominal_6s"]["no_load"] / 3.6
    return {
        "ratio": n,
        **INPUTS["ratios"][f"{n:g}"],
        "top_speed_kmh_pwm100": top,
        "speed_per_1pct_pwm_mps_nominal": round(no_load_top / 100, 4),
        "traction_limit_motor_a": round(motor_current(TRACTION_N, n), 2),
        "cruise_motor_a": round(motor_current(INPUTS["cruise_resistance_n"], n), 2),
        # 같은 근사에서 이 정도 제동 전류면 바퀴 제동력이 이미 접지 한계에 닿는다.
        "brake_current_for_traction_limit_a": round(TRACTION_N * RADIUS / (M["kt_nm_per_a"] * n * ETA), 2),
        "speed_checks": speed_checks,
        "encoder_motor_shaft": encoder,
        "equivalent_ratio_at_60mm": round(n * 0.060 / D, 2),
    }


def main() -> int:
    nominal_no_load_rpm = MOTOR["no_load_rpm"] * INPUTS["battery_v"]["nominal_6s"] / MOTOR["voltage_v"]
    results = {
        "date_kst": "2026-10-02",
        "kind": "근사 계산. 실측·실차 검증 아님",
        "motor_points": MOTOR,
        "motor_model": {k: round(v, 5) for k, v in M.items()},
        "inputs": INPUTS,
        "traction_limit_force_n": round(TRACTION_N, 2),
        "deadband_duty_motor_only_nominal": round(M["i0_a"] * M["resistance_ohm"] / INPUTS["battery_v"]["nominal_6s"], 4),
        "ratio_for_70pct_headroom": {f"{kmh:g}": round(0.7 * nominal_no_load_rpm * math.pi * D / (60 * kmh / 3.6), 2)
                                     for kmh in (10, 12, 15)},
        "archive_60mm_candidate_range_at_65mm": [round(9 * D / 0.060, 2), round(12.5 * D / 0.060, 2)],
        "cases": [evaluate(n) for n in (15.0, 12.0, 2.5)],
        "footprint": {
            "straight_length_with_provisional_wheelbase_m": round(INPUTS["provisional_wheelbase_m"] + D, 3),
            "max_wheelbase_for_length_m": round(INPUTS["footprint_m"]["length"] - D, 3),
            "max_wheel_width_with_provisional_track_m": round(INPUTS["footprint_m"]["width"]
                                                              - INPUTS["provisional_track_width_m"], 3),
        },
    }
    OUT.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for c in results["cases"]:
        t = c["top_speed_kmh_pwm100"]
        s9, s10 = c["speed_checks"]["9"], c["speed_checks"]["10"]
        print(f"{c['ratio']:g}:1  최고속도 공칭 무부하 {t['nominal_6s']['no_load']} / 20 V 접지 한계 "
              f"{t['near_cutoff']['traction_limit']} km/h | 9 km/h duty {s9['duty_no_load']['nominal_6s']:.1%} "
              f"(20 V 접지 한계 {s9['duty_traction_limit']['near_cutoff']:.1%}) | 10 km/h duty 20 V 접지 한계 "
              f"{s10['duty_traction_limit']['near_cutoff']:.1%} | 고정 PWM 저하 "
              f"{s9['fixed_pwm_drop_traction_limit']['drop_fraction']:.1%} | 엔코더 7 PPR "
              f"{c['encoder_motor_shaft']['7']['mm_per_count']} mm")
    print(f"결과: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
