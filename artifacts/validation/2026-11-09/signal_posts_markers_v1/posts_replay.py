"""r25 저장 입력에 STL 기반 양지주 반환을 합성해 같은 계획기 판단을 비교한다(평가 전용 저장 재생).

- 지주 형상은 launch의 _signal_posts와 같은 함수에서 읽는다.
- 합성 반환은 평가기 정답 자세로 만든 시험 입력이며 제어기에 정답을 주는 것이 아니다.
- 열린 고리 재생이라 실제 주행이 달라질 수 있다. 실제 폐루프는 B 주행으로 확인한다.
- 같은 방식으로 공식 벽 상자를 광선 투사해 기록 거리와 비교하여 자세·각도 규약을 점검한다.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

from replay_tools import (HERE, R25, WORLD, Poses, build_controller, digest, finite_ranges, load_launch,
                          output_row, ray_box, read_events, scan_geometry)


def post_boxes():
    root = ET.parse(WORLD).getroot()
    report = load_launch()._signal_posts(root.find("world"), "stl_61cm_provisional", "official_v2026_11_06")
    boxes = [(item["center_xy_m"][0], item["center_xy_m"][1], report["beam_yaw_rad"], item["along_m"], item["across_m"])
             for item in report["links"] if item["name"].startswith("tl_post_")]
    return boxes, report


def wall_boxes():
    """평면 충돌 상자(벽). 규약 점검 전용."""
    model = ET.parse(WORLD).getroot().find("./world/model[@name='it_arena_track_static']")
    boxes = []
    for link in model.findall("link"):
        if link.get("name") != "walls":
            continue
        for col in link.findall("collision"):
            size = col.findtext("geometry/box/size")
            pose = col.findtext("pose")
            if size and pose:
                sx, sy, sz = map(float, size.split())
                x, y, z, _, _, yaw = map(float, pose.split())
                if z - sz / 2 < .075 < z + sz / 2:
                    boxes.append((x, y, yaw, sx, sy))
    return boxes


def main():
    source = R25 / "follow_decisions.jsonl"
    before = digest(source)
    poses = Poses()
    boxes, report = post_boxes()
    walls = wall_boxes()
    injected_scans, wall_checks = {}, []
    base = inj = None
    rows, changed, injected_calls = [], 0, 0
    for event in read_events(source):
        data = event["data"]
        if event["kind"] == "header":
            base, mc = build_controller(data)
            inj, _ = build_controller(data)
            continue
        if event["kind"] == "reset":
            base.reset(); inj.reset()
            continue
        if event["kind"] != "call":
            continue
        from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
        scan = data["input"]["scan"]
        key = (scan["sec"], scan["nanosec"])
        if key not in injected_scans:
            ox, oy, dx, dy = scan_geometry(scan, poses)
            post = np.min([ray_box(ox, oy, dx, dy, *box) for box in boxes], axis=0)
            ranges = finite_ranges(scan["ranges"])
            # 기록이 유한한 광선에서 지주가 더 가까울 때만 바꾼다. 기록상 무반환(후방 사각·자차 구조
            # 마스크 등)은 그대로 둔다(superseded_masked_injection/: 마스크 광선까지 채운 첫 합성 오류).
            replace = np.isfinite(post) & (post <= scan["range_max"]) & (post >= scan["range_min"]) & \
                np.isfinite(ranges) & (post < ranges)
            new_ranges = [float(post[i]) if replace[i] else value for i, value in enumerate(scan["ranges"])]
            injected_scans[key] = (new_ranges, int(replace.sum()), float(np.min(post)) if np.isfinite(post).any() else None)
            if len(wall_checks) < 40 and len(injected_scans) % 7 == 1:
                wall = np.min([ray_box(ox, oy, dx, dy, *box) for box in walls], axis=0)
                ok = np.isfinite(ranges) & np.isfinite(wall) & (wall < 3.0)
                if ok.sum():
                    error = np.abs(ranges[ok] - wall[ok])
                    wall_checks.append(dict(stamp=key[0] + key[1] * 1e-9, rays=int(ok.sum()),
                                            median_abs_m=float(np.median(error)), p90_abs_m=float(np.percentile(error, 90))))
        new_ranges, count, nearest = injected_scans[key]
        out_base = freeze(replay_call(base, data["input"], mc))
        modified = json.loads(json.dumps(data["input"]))
        modified["scan"]["ranges"] = new_ranges
        out_inj = freeze(replay_call(inj, modified, mc))
        same = equal_output(comparable_decision(out_base), comparable_decision(out_inj))
        changed += not same
        injected_calls += count > 0
        row = output_row(event["seq"], out_inj)
        row.update(t=data["input"]["reference_s"], post_rays=count, nearest_post_m=nearest,
                   base_speed=float(out_base[0]), base_steer=float(out_base[1]),
                   base_reason=(out_base[2] or {}).get("reason") if isinstance(out_base[2], dict) else None,
                   identical=same)
        rows.append(row)
    assert digest(source) == before
    with open(HERE / "posts_replay_rows.jsonl", "w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")
    hit = [row for row in rows if row["post_rays"] > 0]
    diff = [row for row in rows if not row["identical"]]
    summary = dict(
        scope="open-loop stored r25 inputs with synthetic STL-derived post returns; planner only (independent guard not replayed)",
        source=str(source.relative_to(HERE.parents[3])).replace("\\", "/"), source_sha256=before,
        posts=report["links"], calls=len(rows), calls_with_post_rays=len(hit), decisions_changed=len(diff),
        post_ray_windows_s=[[hit[0]["t"], None]] if hit else [],
        first_post_time_s=hit[0]["t"] if hit else None, last_post_time_s=hit[-1]["t"] if hit else None,
        min_speed_with_posts_in_hit_calls=min((r["speed"] for r in hit), default=None),
        min_base_speed_in_hit_calls=min((r["base_speed"] for r in hit), default=None),
        max_abs_speed_delta=max((abs(r["speed"] - r["base_speed"]) for r in rows), default=0.),
        max_abs_steer_delta=max((abs(r["steer"] - r["base_steer"]) for r in rows), default=0.),
        zero_speed_new=sum(r["speed"] <= 1e-6 < r["base_speed"] for r in rows),
        changed_reasons=sorted({(r["base_reason"], r["reason"]) for r in diff if r["base_reason"] != r["reason"]}),
        wall_convention_check=dict(scans=len(wall_checks),
                                   median_of_median_abs_m=float(np.median([w["median_abs_m"] for w in wall_checks])) if wall_checks else None,
                                   worst_p90_abs_m=max((w["p90_abs_m"] for w in wall_checks), default=None)),
    )
    # 시간 구간 정리
    windows, start, last = [], None, None
    for row in hit:
        if start is None or row["t"] - last > .5:
            if start is not None:
                windows.append([start, last])
            start = row["t"]
        last = row["t"]
    if start is not None:
        windows.append([start, last])
    summary["post_ray_windows_s"] = windows
    (HERE / "posts_replay_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                                                   encoding="utf-8", newline="\n")
    print(json.dumps(summary, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    sys.exit(main())
