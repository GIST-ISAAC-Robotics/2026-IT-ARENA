#!/usr/bin/env python3
"""고정 속도 LiDAR 시험을 차례로 실행합니다. 기존 결과 폴더를 덮어쓰지 않습니다."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys

from validate_lidar_control_lab import CASES, REPO


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", choices=CASES, nargs="+", default=list(CASES))
    parser.add_argument("--rates", type=float, nargs="+", default=[10., 20., 30., 40.])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if len(set(args.rates)) != len(args.rates) or any(not math.isfinite(rate) or not .5 <= rate <= 100 for rate in args.rates):
        raise ValueError("갱신률은 중복 없이 0.5~100 Hz여야 합니다.")
    if len(set(args.cases)) != len(args.cases):
        raise ValueError("시험 이름을 중복 지정할 수 없습니다.")
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("결과는 저장소 artifacts 아래에만 생성합니다.")
    output.mkdir(parents=True, exist_ok=False)
    summary = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "rates": args.rates,
               "cases": args.cases, "results": []}

    def save():
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    save()
    for case in args.cases:
        for rate in args.rates:
            directory = output / case / (f"{rate:06.2f}".replace(".", "p") + "hz")
            print(f"\nLiDAR lab: {case}, {rate:g} Hz", flush=True)
            completed = subprocess.run([sys.executable, str(REPO / "scripts/validate_lidar_control_lab.py"),
                "--case", case, "--rate", str(rate), "--output", str(directory)], cwd=REPO)
            path = directory / "report.json"
            report = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {"passed": False, "error": "보고서 미생성"}
            summary["results"].append({"case": case, "rate_hz": rate, "return_code": completed.returncode,
                "report": str(path.relative_to(output)), "passed": report["passed"],
                "measurement_completed": report.get("measurement_completed", False),
                "error": report.get("error"), "early_stop_reason": report.get("early_stop_reason"),
                "metrics": report.get("metrics")})
            save()
    summary["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    save()
    # 추종 실패는 실험 결과입니다. 측정 자체가 끝나지 않은 경우에만 반복 실행기를 실패시킵니다.
    return 0 if all(row["measurement_completed"] for row in summary["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
