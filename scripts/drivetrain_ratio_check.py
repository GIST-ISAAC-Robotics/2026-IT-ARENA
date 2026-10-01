"""MB4266-24180 브러시 DC 모터의 전체 감속비 검토 계산.

판매처 사양의 무부하점·정격점 두 개로 선형 DC 모터 모델을 만들고, 바퀴 지름과
전체 감속비(모터 → 바퀴)에 따른 속도 범위·가속 전류·고정 PWM 속도 저하·단락 제동·
엔코더 분해능을 계산한다. 실측값이 아니며 차량 질량·마찰계수·효율은 가정값이다.

예:
    python3 scripts/drivetrain_ratio_check.py
    python3 scripts/drivetrain_ratio_check.py --wheel-mm 72 --ratio 2.5 9.5 --design-top-kmh 15
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math

G = 9.80665


@dataclass(frozen=True)
class MotorPoints:
    """판매처 상세 사양 판독값(24 V 기준)."""

    voltage_v: float = 24.0
    no_load_rpm: float = 18000.0
    no_load_a: float = 2.0
    rated_rpm: float = 16474.0
    rated_a: float = 10.28
    rated_torque_kgfcm: float = 1.079


@dataclass(frozen=True)
class Vehicle:
    """차량·구동 가정값. 실측 전 계산 예시용."""

    mass_kg: float = 2.0
    rear_weight_fraction: float = 0.5
    mu: float = 1.0
    gear_efficiency: float = 0.85
    encoder_ppr: int = 7
    battery_v: float = 22.2
    driver_limit_a: float = 60.0
    control_period_s: float = 0.02


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * 2 * math.pi / 60


def motor_model(points: MotorPoints = MotorPoints()) -> dict:
    """두 운전점에서 V = I·R + Ke·ω, T = Kt·(I − I0)를 푼다."""
    w0 = rpm_to_rad_s(points.no_load_rpm)
    wr = rpm_to_rad_s(points.rated_rpm)
    v = points.voltage_v
    resistance = v * (w0 - wr) / (points.rated_a * w0 - points.no_load_a * wr)
    ke = (v - points.no_load_a * resistance) / w0
    rated_torque = points.rated_torque_kgfcm * G / 100
    kt = rated_torque / (points.rated_a - points.no_load_a)
    stall_a = v / resistance
    stall_torque = kt * (stall_a - points.no_load_a)
    return {
        "resistance_ohm": resistance,
        "ke_v_s_per_rad": ke,
        "kt_nm_per_a": kt,
        "stall_a_at_rated_voltage": stall_a,
        "stall_torque_nm": stall_torque,
        "stall_torque_kgfcm": stall_torque * 100 / G,
        # 같은 부하 토크에서 PWM과 무관하게 떨어지는 회전수
        "speed_drop_rpm_per_nm": points.no_load_rpm / stall_torque,
    }


def battery_no_load_rpm(points: MotorPoints, vehicle: Vehicle) -> float:
    return points.no_load_rpm * vehicle.battery_v / points.voltage_v


def recommended_ratio(wheel_d_m: float, design_top_kmh: float, points: MotorPoints = MotorPoints(),
                      vehicle: Vehicle = Vehicle(), headroom: float = 0.7) -> float:
    """설계 최고속도에서 모터가 배터리 전압 무부하 회전수의 headroom 비율로 돌게 하는 감속비."""
    if not 0 < headroom <= 1:
        raise ValueError("headroom must be within (0, 1]")
    v = design_top_kmh / 3.6
    return headroom * battery_no_load_rpm(points, vehicle) * math.pi * wheel_d_m / (60 * v)


def evaluate(ratio: float, wheel_d_m: float, points: MotorPoints = MotorPoints(),
             vehicle: Vehicle = Vehicle(), speeds_kmh=(9.0, 15.0, 20.0)) -> dict:
    if ratio <= 0 or wheel_d_m <= 0:
        raise ValueError("ratio and wheel diameter must be positive")
    model = motor_model(points)
    r = wheel_d_m / 2
    n0 = battery_no_load_rpm(points, vehicle)
    eta = vehicle.gear_efficiency
    kt, ke, res = model["kt_nm_per_a"], model["ke_v_s_per_rad"], model["resistance_ohm"]

    def motor_rpm(kmh: float) -> float:
        return kmh / 3.6 / (math.pi * wheel_d_m) * 60 * ratio

    traction_n = vehicle.mu * vehicle.rear_weight_fraction * vehicle.mass_kg * G
    traction_motor_nm = traction_n * r / (ratio * eta)
    traction_a = points.no_load_a + traction_motor_nm / kt
    drop_rpm = traction_motor_nm * model["speed_drop_rpm_per_nm"]

    speeds = {}
    for kmh in speeds_kmh:
        n = motor_rpm(kmh)
        w = rpm_to_rad_s(n)
        brake_a = ke * w / res
        speeds[f"{kmh:g}"] = {
            "motor_rpm": n,
            "fraction_of_battery_no_load": n / n0,
            # 해당 속도의 무부하 PWM에 접지 한계 가속 부하를 걸었을 때 상대 저하
            "fixed_pwm_drop_fraction_at_traction_load": drop_rpm / (n + drop_rpm),
            # PWM 0(출력 단락 제동) 순간의 근사 전류와 바퀴 제동력
            "short_brake_a": min(brake_a, vehicle.driver_limit_a),
            "short_brake_wheel_n": kt * min(brake_a, vehicle.driver_limit_a) * ratio * eta / r,
        }

    counts_per_m = vehicle.encoder_ppr * 4 * ratio / (math.pi * wheel_d_m)
    return {
        "ratio": ratio,
        "wheel_d_mm": wheel_d_m * 1000,
        "battery_v": vehicle.battery_v,
        "no_load_top_kmh": n0 / ratio / 60 * math.pi * wheel_d_m * 3.6,
        "traction_force_n": traction_n,
        "traction_limit_motor_a": traction_a,
        "speeds": speeds,
        "encoder_counts_per_wheel_rev": vehicle.encoder_ppr * 4 * ratio,
        "encoder_mm_per_count": 1000 / counts_per_m,
        "speed_step_mps_per_count": 1 / counts_per_m / vehicle.control_period_s,
        # 바퀴가 막혀 역기전력이 없을 때, 기어비와 무관
        "blocked_current_a_by_duty": {
            f"{d:g}": min(d * vehicle.battery_v / res, vehicle.driver_limit_a) for d in (0.1, 0.25, 0.5, 1.0)
        },
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wheel-mm", type=float, nargs="+", default=[50.0, 65.0, 80.0])
    parser.add_argument("--ratio", type=float, nargs="+", default=[2.5])
    parser.add_argument("--design-top-kmh", type=float, nargs="+", default=[20.0, 15.0, 12.0])
    parser.add_argument("--mass-kg", type=float, default=Vehicle.mass_kg)
    parser.add_argument("--rear-weight", type=float, default=Vehicle.rear_weight_fraction)
    parser.add_argument("--mu", type=float, default=Vehicle.mu)
    parser.add_argument("--gear-efficiency", type=float, default=Vehicle.gear_efficiency)
    parser.add_argument("--encoder-ppr", type=int, default=Vehicle.encoder_ppr)
    parser.add_argument("--battery-v", type=float, default=Vehicle.battery_v)
    parser.add_argument("--json", action="store_true", help="표 대신 JSON 출력")
    args = parser.parse_args(argv)

    points = MotorPoints()
    vehicle = Vehicle(mass_kg=args.mass_kg, rear_weight_fraction=args.rear_weight, mu=args.mu,
                      gear_efficiency=args.gear_efficiency, encoder_ppr=args.encoder_ppr,
                      battery_v=args.battery_v)
    model = motor_model(points)
    cases = [evaluate(n, d / 1000, points, vehicle) for d in args.wheel_mm for n in args.ratio]
    rec = {f"{t:g}": {f"{d:g}": recommended_ratio(d / 1000, t, points, vehicle) for d in args.wheel_mm}
           for t in args.design_top_kmh}
    if args.json:
        print(json.dumps({"motor_points": asdict(points), "vehicle": asdict(vehicle), "model": model,
                          "cases": cases, "recommended_ratio": rec}, ensure_ascii=False, indent=2))
        return 0

    print(f"모델: R={model['resistance_ohm']:.3f} Ω, Kt={model['kt_nm_per_a']:.4f} N·m/A, "
          f"Ke={model['ke_v_s_per_rad']:.4f} V·s/rad, 스톨 {model['stall_a_at_rated_voltage']:.0f} A·"
          f"{model['stall_torque_kgfcm']:.1f} kgf·cm, 속도 저하 {model['speed_drop_rpm_per_nm']:.0f} rpm/(N·m)")
    print("\n| 감속비 | 바퀴 mm | 무부하 최고 km/h | 9 km/h 모터 비율 | 접지 한계 가속 A | 고정 PWM 저하(9 km/h) | "
          "PWM 0 제동력 9/20 km/h N | mm/카운트 | 20 ms 속도 단위 m/s |")
    print("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for c in cases:
        s9, s20 = c["speeds"]["9"], c["speeds"]["20"]
        print(f"| {c['ratio']:g} | {c['wheel_d_mm']:g} | {c['no_load_top_kmh']:.0f} | "
              f"{s9['fraction_of_battery_no_load']*100:.0f}% | {c['traction_limit_motor_a']:.1f} | "
              f"{s9['fixed_pwm_drop_fraction_at_traction_load']*100:.0f}% | "
              f"{s9['short_brake_wheel_n']:.0f}/{s20['short_brake_wheel_n']:.0f} | "
              f"{c['encoder_mm_per_count']:.2f} | {c['speed_step_mps_per_count']:.3f} |")
    print(f"\n접지 한계 힘 {cases[0]['traction_force_n']:.1f} N (μ={vehicle.mu}, 뒷바퀴 하중 "
          f"{vehicle.rear_weight_fraction:.0%}, {vehicle.mass_kg} kg)")
    print("\n권장 전체 감속비(설계 최고속도에서 배터리 무부하 회전수의 70%)")
    print("| 설계 최고속도 | " + " | ".join(f"{d:g} mm" for d in args.wheel_mm) + " |")
    print("|---:|" + "---:|" * len(args.wheel_mm))
    for t, row in rec.items():
        print(f"| {t} km/h | " + " | ".join(f"{v:.1f}" for v in row.values()) + " |")
    blocked = cases[0]["blocked_current_a_by_duty"]
    print("\n바퀴가 막혔을 때 전류(기어비 무관, 드라이버 한도 60 A로 자름): "
          + ", ".join(f"PWM {float(k)*100:.0f}% {v:.0f} A" for k, v in blocked.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
