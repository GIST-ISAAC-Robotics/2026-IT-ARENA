"""갈림길 마커 누락의 저장 재생(평가 전용, 열린 고리). 실제 주행은 D 단독 1랩으로 확인한다.

제어기는 마커 ID로 선호 벽(side)만 정한다(ID30→right, ID0/20/45→left). r25 저장 입력에서
side만 바꿔 같은 계획기의 판단을 비교한다.
  drop_30      : ID30을 못 본 경우(blank_20_30/blank_30 한 랩) — 원래 right였던 구간을 left로.
  stale_right  : 직전 랩의 ID30 기억이 남고 0/45·20을 모두 놓친 경우 — P22 접근·통과 구간을 right로.
열린 고리이므로 판단이 갈라진 뒤의 실제 궤적은 재현하지 않는다.
"""
import json
import sys

from replay_tools import HERE, R25, build_controller, digest, output_row, read_events

SCENARIOS = {
    "drop_30": lambda t, side: "left",
    "stale_right": lambda t, side: "right" if 19.0 <= t <= 26.0 else side,
}


def main():
    from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call
    source = R25 / "follow_decisions.jsonl"
    before = digest(source)
    summary = {}
    for name, rule in SCENARIOS.items():
        base = mod = mc = None
        rows = []
        for event in read_events(source):
            data = event["data"]
            if event["kind"] == "header":
                base, mc = build_controller(data)
                mod, _ = build_controller(data)
                continue
            if event["kind"] == "reset":
                base.reset(); mod.reset()
                continue
            if event["kind"] != "call":
                continue
            t, side = data["input"]["reference_s"], data["input"]["side"]
            changed_input = dict(data["input"], side=rule(t, side))
            out_base = freeze(replay_call(base, data["input"], mc))
            out_mod = freeze(replay_call(mod, changed_input, mc))
            row = output_row(event["seq"], out_mod)
            row.update(t=t, side=changed_input["side"], recorded_side=side, base_speed=float(out_base[0]),
                       base_steer=float(out_base[1]),
                       base_reason=out_base[2].get("reason") if isinstance(out_base[2], dict) else None,
                       wall=out_mod[2].get("path_wall") if isinstance(out_mod[2], dict) else None,
                       identical=equal_output(comparable_decision(out_base), comparable_decision(out_mod)))
            rows.append(row)
        with open(HERE / f"marker_{name}_rows.jsonl", "w", encoding="utf-8", newline="\n") as stream:
            for row in rows:
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
        modified = [r for r in rows if r["side"] != r["recorded_side"]]
        diff = [r for r in rows if not r["identical"]]
        summary[name] = dict(
            calls=len(rows), side_modified_calls=len(modified),
            side_modified_window_s=[modified[0]["t"], modified[-1]["t"]] if modified else None,
            decisions_changed=len(diff),
            changed_window_s=[diff[0]["t"], diff[-1]["t"]] if diff else None,
            new_zero_speed=sum(r["speed"] <= 1e-6 < r["base_speed"] for r in rows),
            new_hold_reasons=sorted({r["reason"] for r in diff if r["speed"] <= 1e-6}),
            min_speed_in_modified=min((r["speed"] for r in modified), default=None),
            min_base_speed_in_modified=min((r["base_speed"] for r in modified), default=None),
            max_abs_steer_delta=max((abs(r["steer"] - r["base_steer"]) for r in rows), default=0.),
            mean_steer_delta_in_modified=(sum(r["steer"] - r["base_steer"] for r in modified) / len(modified)
                                          if modified else None),
            walls_used_in_modified=sorted({str(r["wall"]) for r in modified}))
    assert digest(source) == before
    report = dict(scope="open-loop stored r25 inputs; only the marker-derived preferred wall changes; planner only",
                  source_sha256=before, scenarios=summary)
    (HERE / "marker_side_replay_summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                          encoding="utf-8", newline="\n")
    print(json.dumps(report, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    sys.exit(main())
