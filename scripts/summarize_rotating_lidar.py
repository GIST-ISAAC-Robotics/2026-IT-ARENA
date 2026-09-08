#!/usr/bin/env python3
"""저장된 비교 결과만 읽어 표와 추종 그래프를 생성합니다. 재주행하지 않습니다."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    root = args.directory.resolve()
    reports = [(p.parent, json.loads(p.read_text())) for p in sorted(root.glob("*/report.json"))]
    from validate_lidar_control_lab import summarize
    evaluator_hash = hashlib.sha256(Path(__file__).with_name("validate_lidar_control_lab.py").read_bytes()).hexdigest()
    for folder, report in reports:
        trace = [json.loads(line) for line in (folder / "trace.jsonl").read_text().splitlines()]
        commands = [json.loads(line) for line in (folder / "commands.jsonl").read_text().splitlines()]
        scans = json.loads((folder / "scans.json").read_text())
        original_passed = report["passed"]
        report["metrics"] = summarize(trace, commands, scans, report["configuration"], report["requested_lidar_rate_hz"])
        report["passed"] = bool(report["measurement_completed"] and report.get("sensor_only_control")
            and report["metrics"]["fixed_speed_tracking_passed"] and report["shutdown_clean"]
            and "early_stop_reason" not in report and report["acquisition_verified"])
        audit = {"original_report": "report.json", "original_passed": original_passed,
                 "passed": report["passed"], "metrics": report["metrics"],
                 "evaluation_source_sha256": evaluator_hash,
                 "note": "원문 보존. 조기 중단의 stopping 표본을 active 통계에서 제외하여 재집계."}
        (folder / "phase_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    korean = False
    for candidate in (Path("/mnt/c/Windows/Fonts/malgun.ttf"), Path("C:/Windows/Fonts/malgun.ttf")):
        if candidate.exists():
            font_manager.fontManager.addfont(str(candidate))
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(candidate)).get_name()
            plt.rcParams["axes.unicode_minus"] = False
            korean = True
            break
    table = ["# 10 Hz 순간·순차 취득 비교 결과", "",
             "각 행은 독립 실행 1회입니다. 통과는 목표 속도 ±5% 2초 이상 유지·8초 주행·정지까지 노면 이탈 없음·실제 정지·센서/제어 계측·정상 종료를 모두 요구합니다.", "",
             "표는 원시 기록을 주행/제동 단계별로 재집계한 각 실행의 `phase_audit.json` 기준입니다. 최초 `report.json`은 덮어쓰지 않습니다. 기존 집계기는 8초 이전 조기 중단의 제동 구간을 주행 통계에 포함했습니다.", "",
             "| 조건 | 취득 방식 | 통과 | 실제 최고 km/h | 목표 속도 유지 초 | 주행 중 노면 이탈 표본 | 정지 포함 이탈 표본 | 최대 중심 오차 cm | 센서 검증 | 정상 종료 |",
             "|---|---|---|---:|---:|---:|---:|---:|---|---|"]
    cases = {}
    for folder, report in reports:
        m = report.get("metrics", {})
        case, mode = report["case"], report["acquisition"]
        cases.setdefault(case, []).append((folder, report))
        def number(key):
            value = m.get(key)
            return "—" if value is None else f"{value:.3f}"
        error = m.get("tracking_all_active", {}).get("max_abs_centerline_error_m")
        table.append(f"| [{case}]({folder.name}/phase_audit.json) | {mode} | {'통과' if report.get('passed') else '미통과'} | "
                     f"{number('peak_longitudinal_speed_kmh')} | {number('longest_target_speed_dwell_s')} | "
                     f"{m.get('road_departure_samples_active', '—')} | {m.get('road_departure_samples_including_stop', '—')} | "
                     f"{error * 100 if error is not None else float('nan'):.2f} | {report.get('acquisition_verified')} | {report.get('shutdown_clean')} |")
    table.extend(["", "이탈 표본 수는 이탈 사건 수나 확률이 아닙니다. 출발 오프셋 6 cm가 최대 오차에 포함됩니다. 최고속도 순간 도달과 목표 속도 안정 유지도 구분합니다.", "",
                  "## 실패 시계열과 센서 검증", ""])
    for folder, report in reports:
        audit = json.loads((folder / "acquisition.json").read_text())
        trace = [json.loads(line) for line in (folder / "trace.jsonl").read_text().splitlines()]
        commands = [json.loads(line) for line in (folder / "commands.jsonl").read_text().splitlines()]
        departures = [r for r in trace if r["elapsed_s"] >= 0 and r["road_clearance_m"] < 0]
        exceptions = {}
        for command in commands:
            if command["phase"] == "running" and command["reason"] != "following":
                exceptions.setdefault(command["reason"], command["elapsed_s"])
        error = max((r["max_capture_time_error_s"] for r in audit), default=None)
        drops = max((r["discarded_source_gaps"] for r in audit), default=None)
        first = f"{departures[0]['elapsed_s']:.3f}초" if departures else "없음"
        reasons = ", ".join(f"{k} {v:.3f}초" for k, v in exceptions.items()) or "없음"
        table.append(f"- `{folder.name}`: 첫 노면 이탈 {first}; 최초 제어 예외 {reasons}; "
                     f"원시 취득 시각 오차 최대 {error}초; 원시 공백 폐기 {drops}회.")
    table.extend(["",
                  "## 주행 기록 그래프", "", "그래프는 저장된 물리 관측값입니다. 정답 위치는 평가에만 사용하고 조향 입력에는 사용하지 않았습니다.", ""])
    for case, entries in cases.items():
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
        for folder, report in entries:
            trace = [json.loads(line) for line in (folder / "trace.jsonl").read_text().splitlines()]
            trace = [r for r in trace if r["elapsed_s"] >= 0]
            color = "#dc582a" if report["acquisition"] == "sequential" else "#1677a3"
            mode_label = ("순차 취득" if report["acquisition"] == "sequential" else "순간 스캔(부하 일치)") if korean else report["acquisition"]
            label = mode_label + " " + folder.name.rsplit("_", 1)[-1]
            t = [r["elapsed_s"] for r in trace]
            axes[0].plot(t, [r["truth_longitudinal_mps"] * 3.6 for r in trace], label=label, color=color, alpha=.85)
            axes[1].plot(t, [r["centerline_error_m"] * 100 for r in trace], color=color, alpha=.85)
            axes[2].plot(t, [r["road_clearance_m"] * 100 for r in trace], color=color, alpha=.85)
            stopping = [r for r in trace if r["phase"] == "stopping"]
            if stopping and stopping[0]["elapsed_s"] < 7.95:
                for ax in axes:
                    ax.axvline(stopping[0]["elapsed_s"], color=color, linestyle="--", alpha=.7,
                               label=mode_label + (" 조기 중단" if korean else " early stop"))
        for ax in axes:
            ax.grid(alpha=.25)
            ax.axvline(8, color="gray", linestyle=":", label="예정 종료" if korean else "planned stop")
        axes[0].axhline(entries[0][1]["configuration"]["speed_mps"] * 3.6, color="gray", linestyle="--")
        axes[0].set_ylabel("실제 지면 속도 (km/h)" if korean else "Ground speed (km/h)")
        axes[0].legend(fontsize=8)
        axes[1].set_ylabel("중심선 대비 위치 (cm)" if korean else "Center error (cm)")
        axes[2].set_ylabel("노면 가장자리 여유 (cm)" if korean else "Road clearance (cm)")
        axes[2].axhline(0, color="black", linewidth=1)
        axes[2].set_xlabel("출발 후 시뮬레이션 시간 (초)" if korean else "Simulated time since start (s)")
        case_label = ("직선" if case.startswith("straight") else "반경 1 m 커브") + f" {entries[0][1]['configuration']['speed_mps'] * 3.6:g} km/h"
        fig.suptitle(case_label + " | 10 Hz · 같은 원시 계산 부하 · 운동 보정 없음" if korean else case + " | 10 Hz, matched source load, no deskew")
        filename = case + ".png"
        fig.savefig(root / filename, dpi=150)
        plt.close(fig)
        table.extend([f"### {case}", "", f"![{case}]({filename})", ""])
    (root / "RESULTS.md").write_text("\n".join(table) + "\n", encoding="utf-8")
    print(f"{len(reports)} reports summarized")


if __name__ == "__main__":
    main()
