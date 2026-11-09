"""교통 장면 원시 기록에서 녹색·모드 전이·상대 종/횡 위치를 요약한다(읽기 전용).

사용: python scene_timeline.py <run_dir> [--step 0.5]
상대 위치는 자차 자세 기준 좌표(앞 +x, 왼쪽 +y)로, 기록된 Gazebo 자세만 사용한다.
"""
import argparse
import bisect
import json
import math
from pathlib import Path


def load_rows(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def interp(series, t):
    times = [row[0] for row in series]
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return series[0]
    if i >= len(series):
        return series[-1]
    a, b = series[i - 1], series[i]
    w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return [t, a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * dyaw]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--step", type=float, default=0.5)
    args = parser.parse_args()
    ego = load_rows(args.run / "ego_poses.jsonl")
    actor = load_rows(args.run / "actor_poses.jsonl")
    traj = load_rows(args.run / "trajectory.jsonl")
    green = next((row["t"] for row in traj if row.get("light") == "green"), None)
    print(f"ego {ego[0][0]:.3f}-{ego[-1][0]:.3f} n={len(ego)}  actor {actor[0][0]:.3f}-{actor[-1][0]:.3f} n={len(actor)}  green={green}")
    last = None
    for row in traj:
        auto = row.get("autonomy") or {}
        key = (auto.get("state"), auto.get("mode"), auto.get("behavior"))
        if key != last:
            print(f"  t={row['t']:.3f} state={key}")
            last = key
    t = ego[0][0]
    while t <= ego[-1][0]:
        e = interp(ego, t)
        a = interp(actor, t)
        dx, dy = a[1] - e[1], a[2] - e[2]
        c, s = math.cos(e[3]), math.sin(e[3])
        lon, lat = c * dx + s * dy, -s * dx + c * dy
        print(f"t={t:7.2f} ego=({e[1]:.2f},{e[2]:.2f},{math.degrees(e[3]):6.1f}) actor=({a[1]:.2f},{a[2]:.2f}) rel_lon={lon:+.2f} rel_lat={lat:+.2f}")
        t += args.step


if __name__ == "__main__":
    main()
