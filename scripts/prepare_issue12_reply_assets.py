"""저장한 검출 기록으로 #12 회신 표와 실제 RGB 위 후보 좌표 도식을 생성합니다.

새 촬영/검출/주행 없이 원문을 읽습니다. 사진은 수정하지 않고 SVG에 그대로
내장합니다. --check는 생성 문서와 입력 해시를 읽기 전용으로 다시 대조합니다.
"""
import argparse
import base64
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "artifacts/validation/2026-09-15/official_update"
OUT = BASE / "reply_assets"
DRAFT = ROOT / "docs/track/ISSUE_12_REPLY_DRAFT_2026_09_15.md"
IDS = (0, 20, 30, 45)
DISTANCES = (300, 200, 150, 120, 100, 75)
RATES = ("0.05", "0.02", "0.01", "0.1")
CASES = {
    "raw_route": "배포 원본 PNG · 지지대 없음",
    "face_compare_pbr": "PNG · 동일 조건 대조군",
    "face_compare_cells": "도형 · 동일 조건 대조군",
}


def read_json(name):
    return json.loads((BASE / name).read_text(encoding="utf-8"))


def view_name(target, distance):
    return f"id{target}_{distance}cm"


def result_cell(values, target):
    if not values:
        return "—"
    parts = []
    for marker, count in sorted(Counter(values).items()):
        label = str(marker) if count == 1 else f"{marker}×{count}"
        parts.append(f"**{label}**" if marker == target else label)
    return ", ".join(parts)


def build():
    raw = read_json("detector_probe.json")
    face = read_json("face_detector_probe.json")
    trace = read_json("aruco_internal_trace.json")
    views = {"raw_route": raw["views"]["raw_route"], **face["views"]}
    traces = trace["traces"]
    assert len(traces) == 144
    for row in traces:
        ids = views[row["case"]][row["view"]]["candidates"][
            f'minMarkerDistanceRate_{row["rate"]}'
        ]
        assert row["match"] and ids == row["builtin_ids"] == row["trace_ids"]

    def values(case, target, distance, rate):
        view = views[case][view_name(target, distance)]
        assert view["expected_id"] == target
        return view["candidates"][f"minMarkerDistanceRate_{rate}"]

    summary = {}
    for case in CASES:
        assert len(views[case]) == 24
        summary[case] = {}
        for rate in RATES:
            detections = [
                (target, values(case, target, distance, rate))
                for target in IDS for distance in DISTANCES
            ]
            summary[case][rate] = {
                "target_detected_views": sum(target in ids for target, ids in detections),
                "target_duplicated_views": sum(ids.count(target) > 1 for target, ids in detections),
                "any_id_duplicated_views": sum(len(ids) != len(set(ids)) for _, ids in detections),
                "total_views": 24,
            }
    assert [summary[c]["0.05"]["target_detected_views"] for c in CASES] == [10, 9, 15]
    assert [summary[c]["0.02"]["target_detected_views"] for c in CASES] == [17, 17, 19]

    heading = "| 목표 ID | 3.0 m 전 | 2.0 m 전 | 1.5 m 전 | 1.2 m 전 | 1.0 m 전 | 0.75 m 전 |"
    main = [heading, "|---:|:---:|:---:|:---:|:---:|:---:|:---:|"]
    for target in IDS:
        cells = ["검출" if target in values("raw_route", target, d, "0.02") else "미검출" for d in DISTANCES]
        main.append(f"| {target} | " + " | ".join(cells) + " |")

    totals = ["| 사진 조건 | 0.05 목표 검출 | 0.02 목표 검출 | 0.02에서 목표 ID가 중복된 위치 |",
              "|---|---:|---:|---:|"]
    for case, label in CASES.items():
        a, b = summary[case]["0.05"], summary[case]["0.02"]
        totals.append(f'| {label} | {a["target_detected_views"]}/24 | {b["target_detected_views"]}/24 | {b["target_duplicated_views"]}/24 |')

    tables = []
    for case, label in CASES.items():
        tables += [f"#### {label}", "",
                   "| 목표 ID | 기준값 | 3.0 m 전 | 2.0 m 전 | 1.5 m 전 | 1.2 m 전 | 1.0 m 전 | 0.75 m 전 |",
                   "|---:|---:|---|---|---|---|---|---|"]
        for target in IDS:
            for rate in RATES:
                cells = [result_cell(values(case, target, d, rate), target) for d in DISTANCES]
                tables.append(f"| {target} | {rate} | " + " | ".join(cells) + " |")
        tables.append("")

    row = next(r for r in traces if r["case"] == "raw_route" and
               r["view"] == "id30_150cm" and r["rate"] == .05)
    by_index = {c["index"]: c for c in row["candidates"]}
    candidates = {letter: by_index[index] for letter, index in zip("ABCD", (2, 3, 0, 6))}
    group = next(g for g in row["groups"] if 2 in g["members"])
    assert group["kept"] == 6 and group["decoded_id"] == -1
    assert [candidates[k]["standalone_id"] for k in "ABCD"] == [30, 30, -1, -1]

    def distance(a, b):
        return math.sqrt(sum((x-u)**2+(y-v)**2 for (x, y), (u, v)
                             in zip(a["corners"], b["corners"])) / 4)

    pairs = {}
    for left, right in ("AB", "BC", "AC", "CD"):
        a, b = candidates[left], candidates[right]
        pairs[left + right] = {
            "corner_rms_px": distance(a, b),
            "threshold_005_px": .05 * min(a["contour_length"], b["contour_length"]),
            "threshold_002_px": .02 * min(a["contour_length"], b["contour_length"]),
        }
    assert pairs["AB"]["corner_rms_px"] < pairs["AB"]["threshold_005_px"]
    assert pairs["BC"]["corner_rms_px"] < pairs["BC"]["threshold_005_px"]
    assert pairs["AC"]["corner_rms_px"] > pairs["AC"]["threshold_005_px"]
    assert pairs["CD"]["corner_rms_px"] < pairs["CD"]["threshold_005_px"]

    photo = BASE / "raw_route/id30_150cm.png"
    encoded = base64.b64encode(photo.read_bytes()).decode("ascii")
    colors = dict(A="#04d9ed", B="#ffc745", C="#ff57cb", D="#ff6500")
    polygons = []
    for letter, candidate in candidates.items():
        points = " ".join(f"{x},{y}" for x, y in candidate["corners"])
        if letter == "D":
            # 동일 좌표의 선명한 실선. 테두리나 사진 위 라벨은 쓰지 않습니다.
            polygons.append(f'<polygon points="{points}" fill="none" stroke="{colors[letter]}" stroke-width="0.6" stroke-linejoin="round"/>')
        elif letter == "C":
            polygons.append(f'<polygon points="{points}" fill="none" stroke="{colors[letter]}" stroke-width="0.45"/>')
        else:
            polygons.append(f'<polygon points="{points}" fill="none" stroke="{colors[letter]}" stroke-width="0.35"/>')
    crop = '<svg x="{x}" y="158" width="370" height="370" viewBox="385 72 60 60"><use href="#photo"/>{overlay}</svg>'
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1500" height="990" viewBox="0 0 1500 990">
<defs><image id="photo" width="848" height="480" href="data:image/png;base64,{encoded}" style="image-rendering:pixelated"/></defs>
<style>text{{font-family:'Malgun Gothic','Noto Sans CJK KR',sans-serif;fill:#e8edf7;font-size:22px}} .muted{{fill:#afbdd0;font-size:19px}} .heading{{font-size:26px;font-weight:bold}} .small{{font-size:18px}}</style>
<rect width="1500" height="990" fill="#111a29"/>
<text x="40" y="54" font-size="34" font-weight="bold">실제 실패 영상 위에서 본 ArUco 후보 선택</text>
<text x="40" y="92" class="muted">Gazebo 원본 PNG · ID 30 · 본선 진행 거리 1.5 m 전 · OpenCV 4.6.0 · 기본 기준값 0.05</text>
<text x="40" y="135" class="heading">① 실제 카메라 렌더링 전체</text>
<text x="600" y="135" class="heading">② 원본 확대</text>
<text x="1010" y="135" class="heading">③ 같은 사진 + 실제 후보 좌표</text>
<svg x="40" y="158" width="520" height="294.34" viewBox="0 0 848 480"><use href="#photo"/><rect x="385" y="72" width="60" height="60" fill="none" stroke="#ffc745" stroke-width="2.5"/></svg>
{crop.format(x=600, overlay='')}
{crop.format(x=1010, overlay=''.join(polygons))}
<text x="40" y="492" class="muted">노란 상자: 오른쪽 두 그림의 확대 영역</text>
<text x="40" y="526" class="muted">원본 848×480 · 확대 영역 60×60 픽셀</text>
<text x="40" y="560" class="muted">사진의 밝기·무늬는 변경하지 않았습니다.</text>
<text x="600" y="560" class="muted">픽셀을 그대로 확대했습니다. 선은 오른쪽에만 추가했습니다.</text>
<line x1="40" y1="600" x2="80" y2="600" stroke="{colors['A']}" stroke-width="4"/>
<text x="96" y="607">A · 후보 2: 코드 윤곽 → 단독 해독 ID 30</text>
<line x1="760" y1="600" x2="800" y2="600" stroke="{colors['B']}" stroke-width="4"/>
<text x="816" y="607">B · 후보 3: 부풀어진 윤곽 → 단독 해독 ID 30</text>
<line x1="40" y1="645" x2="80" y2="645" stroke="{colors['C']}" stroke-width="3"/>
<text x="96" y="652">C · 후보 0: 판 외곽 → 단독 해독 실패</text>
<line x1="760" y1="645" x2="800" y2="645" stroke="{colors['D']}" stroke-width="4"/>
<text x="816" y="652" font-weight="bold">D · 후보 6: 가장 큰 외곽 → 최종 선택, 해독 실패</text>
<rect x="40" y="686" width="1420" height="252" rx="15" fill="#1d2b40"/>
<text x="64" y="727" class="heading">A·B·C·D는 각각 검출된 후보이며, 모두 같은 그룹으로 묶였습니다.</text>
<text x="64" y="769">A–B: {pairs['AB']['corner_rms_px']:.2f} px &lt; {pairs['AB']['threshold_005_px']:.2f} px</text>
<text x="520" y="769">B–C: {pairs['BC']['corner_rms_px']:.2f} px &lt; {pairs['BC']['threshold_005_px']:.2f} px</text>
<text x="1000" y="769">C–D: {pairs['CD']['corner_rms_px']:.2f} px &lt; {pairs['CD']['threshold_005_px']:.2f} px</text>
<text x="64" y="809" class="muted">거리 = 대응 꼭짓점의 RMS 거리 · 오른쪽 값 = 0.05 × 두 후보 중 짧은 윤곽의 길이</text>
<text x="64" y="851">A·B·C·D를 같은 그룹으로 분류 → 그중 가장 큰 D만 선택 → D 해독 실패 → 최종 미검출</text>
<text x="64" y="893">0.02에서는 이 연결 기준이 좁아져 코드 후보가 보존되고, 같은 사진에서 ID 30이 검출됐습니다.</text>
<text x="40" y="970" class="small">출처: raw_route/id30_150cm.png + aruco_internal_trace.json · A/B/C/D 좌표와 최종 선택은 추적 기록 그대로입니다.</text>
</svg>'''
    inputs = ["detector_probe.json", "face_detector_probe.json", "aruco_internal_trace.json",
              "raw_route/report.json", "raw_front/marker_30.png", "raw_route/id30_200cm.png",
              "raw_route/id30_150cm.png", "face_comparison.json"]
    manifest = {
        "scope": "기존 RGB/검출 기록 재집계. 새 시뮬레이션 또는 실물 측정 아님.",
        "opencv": raw["opencv"], "trace_matches": 144, "threshold_table_observations": 288,
        "summary": summary, "overlay_source": "raw_route/id30_150cm.png",
        "overlay_candidates": candidates, "overlay_group": group, "pair_distances": pairs,
        "input_sha256": {p: hashlib.sha256((BASE / p).read_bytes()).hexdigest() for p in inputs},
    }
    blocks = {"MAIN_TABLE": "\n".join(main), "SUMMARY_TABLE": "\n".join(totals),
              "FULL_TABLES": "\n".join(tables).rstrip()}
    return blocks, svg, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    blocks, svg, manifest = build()
    previous = DRAFT.read_text(encoding="utf-8")
    updated = previous
    for key, value in blocks.items():
        start, end = f"<!-- BEGIN {key} -->", f"<!-- END {key} -->"
        assert updated.count(start) == updated.count(end) == 1
        before, rest = updated.split(start)
        _, after = rest.split(end)
        updated = before + start + "\n" + value + "\n" + end + after
    files = {
        OUT / "candidate_overlay_id30_150cm.svg": svg + "\n",
        OUT / "verified_tables.md": "\n\n".join(blocks.values()) + "\n",
        OUT / "manifest.json": json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        DRAFT: updated,
    }
    if args.check:
        for path, content in files.items():
            assert path.read_text(encoding="utf-8") == content, f"생성 결과 불일치: {path}"
    else:
        OUT.mkdir(parents=True, exist_ok=True)
        for path, content in files.items():
            path.write_text(content, encoding="utf-8")
    print(json.dumps({"checked": args.check, "trace_matches": 144,
                      "observations": 288, "summary": manifest["summary"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
