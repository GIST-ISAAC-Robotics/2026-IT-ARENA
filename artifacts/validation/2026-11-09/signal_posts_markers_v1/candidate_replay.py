"""두 선택형 후보(narrow_occluder_bridge, junction_continuity)의 r25 저장 입력 비교(평가 전용, 열린 고리).

비교 묶음
  plain        : 원 입력. 후보 ON이 기록 판단과 같은지(무주입 불변) 확인
  posts        : 지주 합성 입력(posts_replay와 같은 합성). 후보 OFF/ON
  drop_30      : ID30 누락(side 전부 left). 후보 OFF/ON
  stale_right  : P22 접근·통과 구간 side=right. 후보 OFF/ON
"""
import json
import sys
import numpy as np

from replay_tools import (HERE, R25, Poses, build_controller, digest, finite_ranges, ray_box, read_events,
                          scan_geometry)
from posts_replay import post_boxes

FLAGS = dict(narrow_occluder_bridge=True, junction_continuity=True)


def controller(header, flags):
    # 기록 헤더의 설정에 후보 플래그만 더해 새로 만든다(생성 시 검증 포함).
    return build_controller(dict(header, follow_config={**header["follow_config"], **flags}))


def main():
    from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call
    source = R25 / "follow_decisions.jsonl"
    before = digest(source)
    poses = Poses()
    boxes, _ = post_boxes()
    cache = {}

    def posts_input(data):
        scan = data["scan"]
        key = (scan["sec"], scan["nanosec"])
        if key not in cache:
            ox, oy, dx, dy = scan_geometry(scan, poses)
            post = np.min([ray_box(ox, oy, dx, dy, *b) for b in boxes], axis=0)
            ranges = finite_ranges(scan["ranges"])
            rep = np.isfinite(post) & np.isfinite(ranges) & (post < ranges) & (post <= scan["range_max"])
            cache[key] = [float(post[i]) if rep[i] else v for i, v in enumerate(scan["ranges"])]
        changed = json.loads(json.dumps(data))
        changed["scan"]["ranges"] = cache[key]
        return changed

    variants = {
        "plain": lambda d: d,
        "posts": posts_input,
        "drop_30": lambda d: dict(d, side="left"),
        "stale_right": lambda d: dict(d, side="right") if 19.0 <= d["reference_s"] <= 26.0 else d,
    }
    report = {}
    for name, transform in variants.items():
        runs = {"off": None, "on": None}
        rows = []
        for event in read_events(source):
            data = event["data"]
            if event["kind"] == "header":
                runs = {"off": controller(data, {}), "on": controller(data, FLAGS)}
                continue
            if event["kind"] == "reset":
                for built, _ in runs.values():
                    built.reset()
                continue
            if event["kind"] != "call":
                continue
            given = transform(data["input"])
            outputs = {k: freeze(replay_call(built, given, mc)) for k, (built, mc) in runs.items()}
            recorded = freeze(data["output"])
            on_detail = outputs["on"][2] if isinstance(outputs["on"][2], dict) else {}
            rows.append(dict(t=data["input"]["reference_s"], off=float(outputs["off"][0]), on=float(outputs["on"][0]),
                             off_steer=float(outputs["off"][1]), on_steer=float(outputs["on"][1]),
                             on_reason=on_detail.get("reason"), selection=on_detail.get("boundary_selection"),
                             bridged=on_detail.get("narrow_occluders_bridged"),
                             on_equals_off=equal_output(comparable_decision(outputs["on"]), comparable_decision(outputs["off"])),
                             off_equals_recorded=equal_output(comparable_decision(outputs["off"]), comparable_decision(recorded))))
        with open(HERE / f"candidate_{name}_rows.jsonl", "w", encoding="utf-8", newline="\n") as stream:
            for row in rows:
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
        report[name] = dict(
            calls=len(rows),
            off_equals_recorded=sum(r["off_equals_recorded"] for r in rows),
            on_equals_off=sum(r["on_equals_off"] for r in rows),
            zero_off=sum(r["off"] <= 1e-6 for r in rows), zero_on=sum(r["on"] <= 1e-6 for r in rows),
            mean_speed_off=float(np.mean([r["off"] for r in rows])), mean_speed_on=float(np.mean([r["on"] for r in rows])),
            junction_selected=sum(r["selection"] == "junction_opposite_continuity" for r in rows),
            bridged_calls=sum(bool(r["bridged"]) for r in rows),
            max_abs_steer_on_minus_off=max(abs(r["on_steer"] - r["off_steer"]) for r in rows))
    assert digest(source) == before
    out = dict(scope="open-loop stored r25 inputs; planner only; candidates default OFF", flags=FLAGS,
               source_sha256=before, variants=report)
    (HERE / "candidate_replay_summary.json").write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n",
                                                       encoding="utf-8", newline="\n")
    print(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    sys.exit(main())
