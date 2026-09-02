#!/usr/bin/env python3
"""완료된 LiDAR 시험 JSON을 읽어 결과표와 속도·횡오차 그래프를 만듭니다."""

import argparse
import json
from pathlib import Path

from validate_lidar_control_lab import CASES, REPO
from lidar_shutdown import audit_shutdown


LABELS = {"straight_20kmh": "직선 · 20 km/h", "circle_5kmh": "반경 1 m 원 · 5 km/h",
          "circle_8kmh": "반경 1 m 원 · 8 km/h"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    if not directory.is_relative_to(REPO / "artifacts"):
        raise ValueError("프로젝트 artifacts 안의 시험만 요약합니다.")
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    if "finished_at_utc" not in summary:
        raise ValueError("반복 시험이 아직 완료되지 않았습니다.")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    korean_font = Path("/mnt/c/Windows/Fonts/malgun.ttf")
    if korean_font.exists():
        font_manager.fontManager.addfont(str(korean_font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(korean_font)).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["font.size"] = 10
    cases = summary["cases"]
    figure, axes = plt.subplots(len(cases), 2, figsize=(13, 3.2 * len(cases)), squeeze=False, constrained_layout=True)
    palette = ["#d1495b", "#276fbf", "#df8b19", "#24844b", "#7d4da8"]
    colors = {rate: palette[index % len(palette)] for index, rate in enumerate(summary["rates"])}
    lines = ["# 고정 속도 LiDAR 조향 시험", "",
        "원본 JSON에서 자동 생성한 표입니다. 속도 도달, 추종·정지, 프로그램 종료를 분리합니다.", "",
        "- 갱신률마다 500점, 동일 차량·100 Hz 조향기·물리 1 ms. 센서 입력은 `/scan`과 `/clock`입니다.",
        "- 속도 유지: 목표 ±5%를 연속 2초 이상. 외형 판정: 회전한 20×17 cm 보수적 상자.",
        "- 노면 폭 45 cm·벽 사이 85 cm. 원형 시험 반경 1 m는 공식 코스의 급커브보다 완만합니다.",
        "- ToF 감속·RGB 출발·스캔 운동 보상·순차 회전 취득은 제외한 시험입니다. 실차 안전 인증이 아닙니다.", "",
        "| 조건 | 설정 Hz | 실측 Hz | 최고 km/h | 속도 유지 s | 주행 RMSE cm¹ | 노면 이탈 표본² | 벽 겹침 표본² | 추종·정지 | 정상 종료 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|:---:|:---:|"]
    shutdown_results = []
    for entry in summary["results"]:
        path = directory / entry["report"]
        report = json.loads(path.read_text(encoding="utf-8"))
        shutdown = audit_shutdown((path.parent / "simulation.log").read_text(encoding="utf-8", errors="replace"),
                                  report.get("launch_return_code"), report.get("forced_cleanup"))
        shutdown_results.append({"report": entry["report"], "original_shutdown_clean": report.get("shutdown_clean"),
            "corrected_overall_passed": bool(report.get("measurement_completed") and report.get("sensor_only_control")
                and report.get("metrics", {}).get("fixed_speed_tracking_passed") and shutdown["clean"]
                and not report.get("early_stop_reason")), **shutdown})
        metrics = report.get("metrics", {})
        rmse = metrics.get("tracking_after_3s", {}).get("centerline_rmse_m")
        value = lambda key, digits=2: "—" if metrics.get(key) is None else f"{metrics[key]:.{digits}f}"
        lines.append(f"| {LABELS[entry['case']]} | [{entry['rate_hz']:g}]({entry['report']}) | "
            f"{value('measured_lidar_rate_hz')} | {value('peak_longitudinal_speed_kmh')} | "
            f"{value('longest_target_speed_dwell_s')} | {'—' if rmse is None else f'{rmse * 100:.2f}'} | "
            f"{value('road_departure_samples_including_stop', 0)} | {value('wall_overlap_samples_including_stop', 0)} | "
            f"{'통과' if metrics.get('fixed_speed_tracking_passed') else '실패'} | {'통과' if shutdown['clean'] else '실패'} |")
        trace_path = path.parent / "trace.jsonl"
        rows = [json.loads(line) for line in trace_path.read_text(encoding="utf-8").splitlines() if line]
        rows = [row for row in rows if row["elapsed_s"] >= 0]
        if not rows:
            continue
        row_index = cases.index(entry["case"])
        elapsed = [row["elapsed_s"] for row in rows]
        label = f"{entry['rate_hz']:g} Hz"
        color = colors[entry["rate_hz"]]
        axes[row_index, 0].plot(elapsed, [row["truth_longitudinal_mps"] * 3.6 for row in rows], label=label, color=color, linewidth=1.5)
        axes[row_index, 1].plot(elapsed, [row["centerline_error_m"] * 100 for row in rows], label=label, color=color, linewidth=1.5)
    for index, case in enumerate(cases):
        target = CASES[case]["speed_mps"] * 3.6
        axes[index, 0].axhspan(target * .95, target * 1.05, color="#666666", alpha=.10)
        axes[index, 0].axhline(target, color="#333333", linestyle="--", linewidth=.8)
        axes[index, 1].axhline(0, color="#333333", linestyle="--", linewidth=.8)
        for column in range(2):
            ax = axes[index, column]
            ax.set_title(LABELS[case] + (" · 실제 지면 속도" if column == 0 else " · 중심선 오차"))
            ax.set_xlabel("시뮬레이션 경과 시간 (s)")
            ax.set_ylabel("속도 (km/h)" if column == 0 else "중심선 오차 (cm)")
            ax.axvline(8., color="#666666", linestyle=":", linewidth=1.)
            ax.grid(alpha=.2)
            ax.legend(loc="best", fontsize=8)
    figure.suptitle("LiDAR 갱신률 비교 · 점선 8초 이후 제동 · 이상적 snapshot 센서", fontsize=13)
    figure.savefig(directory / "speed_and_tracking.png", dpi=160)
    plt.close(figure)
    lines += ["", "정상 종료 열은 자식 프로세스 로그까지 다시 감사한 결과입니다. 초기 schema 1 보고서의 `shutdown_clean`은 부모만 확인했으므로 그대로 사용하지 않았습니다.",
        "[부모·자식 종료 감사 원문](shutdown_audit.json)", "",
        "¹ 출발 후 3초부터 주행 종료 전까지. 출발 오프셋 6 cm와 가속 구간의 영향을 줄인 지표입니다.",
        "² 정지 과정까지 포함한 약 50 Hz 관측 표본 수입니다. 충돌 횟수·사고 횟수가 아니며, 서로 다른 표본 수를 사건 수로 비교하지 않습니다.", "",
        "![시뮬레이션 실제 속도와 중심선 오차](speed_and_tracking.png)", "",
        "스캔 수집 시각과 제어 적용 시각의 관계, 제동 중 거동도 결과에 영향을 줍니다.",
        "고정 속도 한 조건의 통과를 코스 완주·다중 차량·실물 성능이나 최소 구매 사양으로 확대하지 않습니다.", ""]
    (directory / "README.md").write_text("\n".join(lines), encoding="utf-8")
    (directory / "shutdown_audit.json").write_text(json.dumps(shutdown_results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(directory / "README.md")


if __name__ == "__main__":
    main()
