#!/usr/bin/env python3
"""동일 조건의 Gazebo 자율주행을 여러 2D LiDAR 갱신률로 반복하고 요약합니다."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys


REPO = Path(__file__).resolve().parents[1]


def rate_label(rate: float) -> str:
    return f"{rate:06.2f}".replace(".", "p") + "hz"


def result_row(rate: float, report: dict, return_code: int) -> dict:
    tracking = report.get("tracking_metrics", {})
    lidar = report.get("lidar", {})
    return {
        "requested_rate_hz": rate,
        "measured_rate_sim_hz": lidar.get("measured_rate_sim_hz"),
        "return_code": return_code,
        "passed": bool(report.get("passed")),
        "error": report.get("error"),
        "progress_m": report.get("progress_m"),
        "peak_ground_speed_mps": report.get("peak_ground_speed_mps"),
        "peak_ground_speed_kmh": (
            report.get("peak_ground_speed_mps", 0.0) * 3.6
            if report.get("peak_ground_speed_mps") is not None
            else None
        ),
        "max_centerline_error_m": report.get("max_centerline_error_m"),
        "centerline_rmse_m": tracking.get("centerline_rmse_m"),
        "centerline_p95_m": tracking.get("centerline_p95_m"),
        "steering_rms_rad": tracking.get("steering_rms_rad"),
        "steering_max_step_rad": tracking.get("steering_max_step_rad"),
        "sensor_stop_samples": tracking.get("sensor_stop_samples"),
        "distance_per_scan_at_peak_speed_m": lidar.get("distance_per_scan_at_peak_speed_m"),
        "distance_per_scan_at_20kmh_m": lidar.get("distance_per_scan_at_20kmh_m"),
        "idealized_snapshot_model": lidar.get("idealized_snapshot_model"),
        "collision_samples": report.get("collision_samples"),
        "stop_stable": report.get("stop_stable"),
    }


def markdown(summary: dict) -> str:
    lines = [
        "# 2D LiDAR 갱신률 직접 비교",
        "",
        f"- 속도 프로필: `{summary['speed_profile']}`",
        f"- 목표 진행 거리: {summary['target_progress_m'] if summary['target_progress_m'] is not None else '한 바퀴+2 m'}",
        f"- ToF 안전층: {'끔(라이다 조향 분리 스트레스)' if summary['disable_tof_safety'] else '켬'}",
        "- 주의: Gazebo의 동시 광선 snapshot이며 회전 왜곡·전송 지연·반사율 미검출은 재현하지 않습니다.",
        "- PASS는 지정 거리 진행·정지 검사입니다. 프로필 이름의 속도에 실제 도달했다는 뜻이 아닙니다.",
        "- 기본 조향 루프는 20 Hz이고 조향 변화 지표는 약 5 Hz 상태 메시지 표본입니다. 원시 제어 명령의 변화량이 아닙니다.",
        "",
        "| 요청 Hz | 실측 Hz | 통과 | 진행 m | 최고 km/h | 최대 횡오차 m | RMSE m | 최대 조향 변화 rad | 20km/h에서 스캔당 이동 m |",
        "|---:|---:|:---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary["results"]:
        def show(key, digits=3):
            value = row.get(key)
            return "-" if value is None else f"{value:.{digits}f}"
        lines.append(
            f"| {row['requested_rate_hz']:g} | {show('measured_rate_sim_hz', 2)} | "
            f"{'PASS' if row['passed'] else 'FAIL'} | {show('progress_m', 2)} | "
            f"{show('peak_ground_speed_kmh', 2)} | {show('max_centerline_error_m')} | "
            f"{show('centerline_rmse_m')} | {show('steering_max_step_rad')} | "
            f"{show('distance_per_scan_at_20kmh_m')} |"
        )
    lines.extend(["", "실패 행의 원인은 각 주기 하위 폴더의 `report.json`과 `simulation.log`에서 확인합니다.", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rates", type=float, nargs="+", required=True)
    parser.add_argument("--speed-profile", default="brisk",
                        choices=("cautious", "brisk", "exploratory", "hardware_target", "lidar_rate_stress", "lidar_20kmh_straight"))
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--target-progress-m", type=float)
    parser.add_argument("--max-sim-seconds", type=float, default=180)
    parser.add_argument("--red-duration-s", type=float, default=1.5)
    parser.add_argument("--disable-tof-safety", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if (not args.rates or any(not math.isfinite(rate) or not .5 <= rate <= 100 for rate in args.rates)
            or len(set(args.rates)) != len(args.rates)):
        raise ValueError("주기는 중복 없이 0.5~100 Hz의 유한한 값이어야 합니다.")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = (args.output or REPO / "artifacts/tests/lidar_rate_sweep" / stamp).resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("출력은 프로젝트 artifacts 아래에 둡니다.")
    output.mkdir(parents=True, exist_ok=False)
    summary = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "speed_profile": args.speed_profile,
        "rates_hz": args.rates,
        "laps": args.laps,
        "target_progress_m": args.target_progress_m,
        "max_sim_seconds": args.max_sim_seconds,
        "red_duration_s": args.red_duration_s,
        "disable_tof_safety": args.disable_tof_safety,
        "results": [],
    }
    for rate in args.rates:
        case = output / rate_label(rate)
        command = [
            sys.executable,
            str(REPO / "scripts/validate_basic_autonomy.py"),
            "--laps", str(args.laps),
            "--max-sim-seconds", str(args.max_sim_seconds),
            "--speed-profile", args.speed_profile,
            "--lidar-rate-hz", str(rate),
            "--red-duration-s", str(args.red_duration_s),
            "--output", str(case),
        ]
        if args.target_progress_m is not None:
            command.extend(("--target-progress-m", str(args.target_progress_m)))
        if args.disable_tof_safety:
            command.append("--disable-tof-safety")
        print(f"\n=== LiDAR {rate:g} Hz ===", flush=True)
        completed = subprocess.run(command, cwd=REPO)
        report_path = case / "report.json"
        report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {
            "passed": False, "error": "report.json이 생성되지 않았습니다."
        }
        summary["results"].append(result_row(rate, report, completed.returncode))
        (output / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        (output / "README.md").write_text(markdown(summary), encoding="utf-8")
    summary["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output / "README.md").write_text(markdown(summary), encoding="utf-8")
    print(f"\n요약: {output / 'README.md'}", flush=True)
    return 0 if all(row["passed"] for row in summary["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
