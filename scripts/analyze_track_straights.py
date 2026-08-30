#!/usr/bin/env python3
"""폐회로 중심선에서 방향 변화 폭이 작은 최장 연속 구간을 읽기 전용으로 계산합니다."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


TRACK_DIRECTORIES = {"original": "it_arena_track", "experimental": "it_arena_experimental"}


def load_points(path: Path) -> list[tuple[float, float]]:
    with path.open(encoding="utf-8", newline="") as stream:
        points = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
    if len(points) > 1 and math.dist(points[0], points[-1]) < 1e-8:
        points.pop()
    return points


def longest_straight(points: list[tuple[float, float]], tolerance_deg: float) -> dict:
    """구간 내 모든 선분 방위각의 최대-최소 차를 제한합니다. 폐회로 이음부도 검사합니다."""
    if len(points) < 3 or not 0 < tolerance_deg < 90:
        raise ValueError("세 점 이상의 폐회로와 0~90도 사이의 양의 허용 각도가 필요합니다.")
    edges = [(end[0] - start[0], end[1] - start[1])
             for start, end in zip(points, points[1:] + points[:1])]
    lengths = [math.hypot(dx, dy) for dx, dy in edges]
    if not all(math.isfinite(length) and length > 0 for length in lengths):
        raise ValueError("중복점 또는 유효하지 않은 좌표가 있습니다.")
    headings = [math.atan2(dy, dx) for dx, dy in edges]
    tolerance = math.radians(tolerance_deg)
    best_length, best_start, best_count, best_span = 0.0, 0, 0, 0.0
    for start in range(len(points)):
        low = high = 0.0
        distance = 0.0
        for offset in range(len(points)):
            index = (start + offset) % len(points)
            difference = headings[index] - headings[start]
            relative = math.atan2(math.sin(difference), math.cos(difference))
            low, high = min(low, relative), max(high, relative)
            if high - low > tolerance + 1e-12:
                break
            distance += lengths[index]
            if distance > best_length:
                best_length, best_start, best_count, best_span = distance, start, offset + 1, high - low

    end_index = (best_start + best_count) % len(points)
    start, end = points[best_start], points[end_index]
    chord = math.dist(start, end)
    dx, dy = end[0] - start[0], end[1] - start[1]
    deviation = max(abs(dx * (points[(best_start + offset) % len(points)][1] - start[1])
                        - dy * (points[(best_start + offset) % len(points)][0] - start[0])) / chord
                    for offset in range(best_count + 1))
    return {
        "heading_span_tolerance_deg": tolerance_deg,
        "length_m": best_length,
        "chord_length_m": chord,
        "actual_heading_span_deg": math.degrees(best_span),
        "maximum_chord_deviation_m": deviation,
        "start_index": best_start,
        "end_index": end_index,
        "crosses_csv_origin": best_start + best_count >= len(points),
        "start_xy_m": list(start),
        "end_xy_m": list(end),
    }


def analyze_track(repo: Path, track: str) -> dict:
    path = repo / "src/arena_gazebo/worlds" / TRACK_DIRECTORIES[track] / "centerline.csv"
    points = load_points(path)
    return {
        "track": track,
        "centerline_file": path.relative_to(repo).as_posix(),
        "sample_count": len(points),
        "lap_length_from_csv_m": sum(math.dist(a, b) for a, b in zip(points, points[1:] + points[:1])),
        "longest_segments": [longest_straight(points, tolerance) for tolerance in (.25, .75, 1.0)],
        "clear_line_of_sight_verified": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", choices=("both", *TRACK_DIRECTORIES), default="both")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    tracks = list(TRACK_DIRECTORIES) if args.track == "both" else [args.track]
    print(json.dumps([analyze_track(repo, track) for track in tracks], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
