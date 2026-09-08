#!/usr/bin/env python3
"""같은 원시 렌더 부하의 순간/순차 10 Hz 속도 비교. 실패도 결과로 보존."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", default=["straight_5kmh", "straight_10kmh", "straight_15kmh",
                        "straight_20kmh", "circle_5kmh", "circle_8kmh", "circle_10kmh"])
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "artifacts") or args.repeats < 1:
        raise ValueError("출력 경로 또는 반복 수 오류")
    output.mkdir(parents=True, exist_ok=False)
    summary = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "results": []}
    def save():
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    save()
    for case in args.cases:
        for repeat in range(args.repeats):
            for mode in ("snapshot_matched", "sequential"):
                name = f"{case}_{mode}_{repeat+1:02d}"
                directory = output / name
                print(f"START {name}", flush=True)
                with (output / f"{name}.log").open("w", encoding="utf-8") as log:
                    result = subprocess.run([sys.executable, str(REPO / "scripts/validate_lidar_control_lab.py"),
                        "--case", case, "--rate", "10", "--acquisition", mode, "--source-rate", "500",
                        "--output", str(directory)], cwd=REPO, stdout=log, stderr=subprocess.STDOUT)
                path = directory / "report.json"
                report = json.loads(path.read_text()) if path.exists() else {}
                summary["results"].append({"name": name, "return_code": result.returncode,
                    "passed": report.get("passed", False), "measurement_completed": report.get("measurement_completed", False),
                    "acquisition_verified": report.get("acquisition_verified"), "shutdown_clean": report.get("shutdown_clean"),
                    "error": report.get("error"), "metrics": report.get("metrics")})
                save()
                print(f"DONE {name} passed={report.get('passed')} acquisition={report.get('acquisition_verified')}", flush=True)
                if not report.get("measurement_completed") or not report.get("acquisition_verified"):
                    print("STOP: 측정 계통 실패를 제품 주행 성능으로 해석하지 않습니다.", flush=True)
                    return 1
    summary["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    save()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
