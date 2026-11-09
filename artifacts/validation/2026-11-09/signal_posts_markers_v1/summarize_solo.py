"""단독 1랩 기록 요약(평가 전용). r25 live_summary와 같은 정의에 신호 지주 구간·갈림길 구간 지표를 더한다.

사용: python summarize_solo.py <run_dir> [<run_dir> ...]  (Windows/WSL python, numpy 불필요)
"""
import json
import math
import sys
from pathlib import Path

SIGNAL_S, LAP = 44.285, 44.64
P21, P22 = (30.315, 33.725), (19.06, 23.89)


def speed(row):
    return abs(float(row["dynamics"]["truth_longitudinal_mps"]))


def summarize(run):
    run = Path(run)
    report = json.loads((run / "report.json").read_text(encoding="utf-8"))
    trace = json.loads((run / "trace.json").read_text(encoding="utf-8"))
    start = report.get("official_signal_evidence", {}).get("first_start_sim_s")
    target = report.get("target_progress_m")
    goal = next((r for r in trace if r["progress_m"] >= target - 2.0 + 2.0 - 1e-9), None) if target else None
    # r25 정의: 목표 진행(랩+2 m 요청 중 실제 46.67 m 도달) 표본. 여기서는 마지막 정지 전 표본을 쓴다.
    last = trace[-1]
    pairs = list(zip(trace[:-1], trace[1:]))
    fast = sum(b["time"] - a["time"] for a, b in pairs if speed(a) >= 10 / 3.6)
    low = [(a["time"], a["progress_m"], speed(a)) for a in trace
           if a["progress_m"] > .5 and speed(a) < .05 and start is not None and a["time"] > start]
    low_sum = sum(b["time"] - a["time"] for a, b in pairs
                  if a["progress_m"] > .5 and speed(a) < .05 and start is not None and a["time"] > start)
    # 가장 긴 연속 저속(0.05 m/s 미만) 구간
    longest, current = 0., 0.
    for a, b in pairs:
        if start is not None and a["time"] > start and a["progress_m"] > .5 and speed(a) < .05:
            current += b["time"] - a["time"]
            longest = max(longest, current)
        else:
            current = 0.

    def window(lo, hi, lap_index=None):
        rows = [r for r in trace if lo <= r["s"] <= hi and r["time"] > (start or 0) and
                (lap_index is None or int((r["progress_m"]) // LAP) == lap_index)]
        if not rows:
            return None
        return dict(samples=len(rows), min_mps=round(min(map(speed, rows)), 3), mean_mps=round(sum(map(speed, rows)) / len(rows), 3),
                    duration_s=round(rows[-1]["time"] - rows[0]["time"], 3))

    post_start = [r for r in trace if r["progress_m"] < 5 and abs(r["s"] - SIGNAL_S) <= 1.0 and r["time"] > (start or 0)]
    post_end = [r for r in trace if r["progress_m"] > 30 and abs(r["s"] - SIGNAL_S) <= 1.0]
    def seg(rows):
        if not rows:
            return None
        return dict(samples=len(rows), min_mps=round(min(map(speed, rows)), 3),
                    mean_mps=round(sum(map(speed, rows)) / len(rows), 3),
                    duration_s=round(rows[-1]["time"] - rows[0]["time"], 3),
                    progress_m=[round(rows[0]["progress_m"], 2), round(rows[-1]["progress_m"], 2)])
    stop = json.loads((run / "stop_trace.json").read_text(encoding="utf-8")) if (run / "stop_trace.json").exists() else []
    sides = []
    for r in trace:
        side = r["autonomy"].get("side")
        if not sides or sides[-1][1] != side:
            sides.append((round(r["time"], 2), side, round(r["s"], 2)))
    return dict(
        run=run.as_posix(), passed=report.get("passed"), source_unchanged=report.get("source_unchanged"),
        progress_m=report.get("progress_m"), completed_laps=report.get("completed_laps"),
        collision_samples=report.get("collision_samples"), stop_collision_samples=report.get("stop_collision_samples"),
        stop_stable=report.get("stop_stable"), route_complete=report.get("route_complete"),
        shutdown_clean=report.get("shutdown_clean"), marker_ids=report.get("marker_ids"),
        race_start_sim_s=start, last_pre_stop_trace_s=last["time"],
        elapsed_to_last_sample_s=None if start is None else round(last["time"] - start, 3),
        peak_ground_speed_mps=report.get("peak_ground_speed_mps"),
        sampled_10kmh_time_s=round(fast, 3), low_sample_interval_sum_s=round(low_sum, 3),
        longest_low_interval_s=round(longest, 3), low_samples=low[:12],
        max_centerline_error_m=report.get("max_centerline_error_m"),
        signal_post_window_start=seg(post_start), signal_post_window_end=seg(post_end),
        p22_window=window(*P22), p21_window=window(*P21), side_changes=sides,
        signal_post_evidence={k: v for k, v in (report.get("signal_post_evidence") or {}).items() if k != "links"} or None,
        marker_visibility_evidence=report.get("marker_visibility_evidence"),
        stop_displacement_m=report.get("stop_displacement_m"), stop_wait_sim_s=report.get("stop_wait_sim_s"),
        stop_samples=len(stop))


if __name__ == "__main__":
    out = [summarize(path) for path in sys.argv[1:]]
    print(json.dumps(out, ensure_ascii=False, indent=1))
