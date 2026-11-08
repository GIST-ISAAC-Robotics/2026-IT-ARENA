#!/usr/bin/env python3
"""공식 코스 녹화의 실제 속도·센서 상태·실측 궤적을 별도 집계합니다."""
import argparse
from collections import Counter
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from validate_basic_autonomy import DEMO_TRACK, TRACK_WORLDS

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--track", choices=tuple(TRACK_WORLDS), default=DEMO_TRACK,
                        help="궤적 겹침용 평가 중심선 월드. report.json의 track과 같아야 합니다.")
    args = parser.parse_args()
    folder = args.run.resolve()
    if not folder.is_relative_to(REPO / "artifacts"):
        raise ValueError("artifacts 내부 실행만 검사합니다.")
    output = folder / "lap_analysis.json"
    if output.exists():
        raise FileExistsError(output)
    report = json.loads((folder / "report.json").read_text())
    recorded = report.get("track", DEMO_TRACK)
    if recorded in TRACK_WORLDS and recorded != args.track:
        raise ValueError(f"report.json track={recorded}와 --track={args.track}이 다릅니다.")
    trace = json.loads((folder / "trace.json").read_text())
    active = [r for r in trace if r["autonomy"].get("started")]
    if not active:
        raise ValueError("주행 표본이 없습니다.")
    t = np.array([r["time"] for r in active])
    speed = np.array([abs(r["dynamics"]["truth_longitudinal_mps"]) * 3.6 for r in active])
    stamps = np.diff(t)
    statuses = {r["autonomy"]["sim_time_s"]: r["autonomy"] for r in active}
    result = {"source_report_passed": report["passed"], "progress_m": report.get("progress_m"),
              "completed_laps": report.get("completed_laps"), "peak_speed_kmh": float(max(speed)),
              "time_at_or_above_19kmh_s": float(sum(stamps[speed[:-1] >= 19.])),
              "time_at_or_above_20kmh_s": float(sum(stamps[speed[:-1] >= 20.])),
              "active_span_sim_s": float(t[-1] - t[0]),
              "time_weighted_mean_speed_kmh": float(np.sum(stamps * speed[:-1]) / sum(stamps)),
              "status_samples": dict(Counter(r["state"] for r in statuses.values())),
              "motion_reasons": dict(Counter(r.get("motion", {}).get("reason", "missing") for r in statuses.values())),
              "max_abs_roll_deg": float(max(abs(r["dynamics"]["truth_roll_rad"]) for r in active) * 180 / np.pi),
              "max_abs_pitch_deg": float(max(abs(r["dynamics"]["truth_pitch_rad"]) for r in active) * 180 / np.pi),
              "scope": "sampled simulator truth used only for evaluation; longitudinal speed is not commanded or wheel speed"}
    if args.track != DEMO_TRACK:
        result["track"] = args.track
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    route = np.genfromtxt(TRACK_WORLDS[args.track] / "centerline.csv", delimiter=",", names=True)
    fig, ax = plt.subplots(1, 2, figsize=(12, 7), gridspec_kw={"width_ratios": [1, 1.1]})
    ax[0].plot(route['x_m'], route['y_m'], color='gray', alpha=.5, label='Reference centerline (evaluation only)')
    dots = ax[0].scatter([r['x'] for r in active], [r['y'] for r in active], c=speed, s=6, vmin=0, vmax=20, cmap='turbo')
    ax[0].set_aspect('equal')
    course = "Official course" if args.track == DEMO_TRACK else f"Official course {args.track}"
    ax[0].set_title(f"{course} / completed laps: {result['completed_laps']}")
    fig.colorbar(dots, ax=ax[0], label='Actual speed [km/h]')
    ax[1].plot(t - t[0], speed)
    ax[1].axhline(20, color='gray', linestyle='--', label='20 km/h target ceiling')
    ax[1].set_xlabel('Elapsed simulation time [s]')
    ax[1].set_ylabel('Actual longitudinal speed [km/h]')
    ax[1].legend()
    for axis in ax:
        axis.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(folder / 'lap_speed_and_trajectory.png', dpi=130)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
