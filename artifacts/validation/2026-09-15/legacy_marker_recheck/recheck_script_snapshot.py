#!/usr/bin/env python3
"""#12 최초 제보의 저장 RGB를 계수만 바꿔 재검사합니다. 월드/운영 코드는 불변입니다."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import cv2


REPO = Path(__file__).resolve().parents[1]
SOURCE = REPO / "artifacts/validation/2026-09-01/official_update/marker_vehicle_view"
REPORTS = {"default": "default_cases_report.json", "dense": "dense_sweep_report.json"}
RATES = (0.05, 0.02, 0.01, 0.1)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parameters_dict(params):
    return {key: getattr(params, key) for key in dir(params)
            if not key.startswith("_") and isinstance(getattr(params, key), (int, float, bool))}


def detect(gray, dictionary, params=None):
    if params is None:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return {"ids": [] if ids is None else ids.flatten().tolist(),
            "corners_px": [quad.reshape(4, 2).tolist() for quad in corners],
            "rejected_candidates": len(rejected)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="artifacts/validation/2026-09-15/legacy_marker_recheck")
    args = parser.parse_args()
    output = (REPO / args.output_dir).resolve()
    relative = output.relative_to(REPO / "artifacts/validation")
    if len(relative.parts) < 2 or output.exists():
        raise ValueError("검증 폴더 아래의 새로운 하위 경로만 출력 대상으로 허용합니다.")
    if cv2.__version__ != "4.6.0":
        raise RuntimeError(f"당시와 같은 OpenCV 4.6.0이 필요합니다: {cv2.__version__}")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    defaults = parameters_dict(cv2.aruco.DetectorParameters_create())
    assert defaults["minMarkerDistanceRate"] == 0.05
    report = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "공식 v2026.09.01 벽 평행 PNG 마커의 과거 저장 RGB 재검사. 새 촬영/주행/실물 시험 아님.",
        "opencv": cv2.__version__, "opencv_module": cv2.__file__,
        "dictionary": "DICT_4X4_50", "rates": RATES, "default_parameters": defaults,
        "preprocessing": "원본 PNG를 BGR로 읽은 뒤 COLOR_BGR2GRAY. 밝기/크기/각도 보정 없음.",
        "script_sha256": digest(Path(__file__)), "sources": {}, "views": [],
    }
    copies = []
    for group, filename in REPORTS.items():
        source_path = SOURCE / filename
        source = json.loads(source_path.read_text(encoding="utf-8"))
        report["sources"][group] = {"report": source_path.relative_to(REPO).as_posix(),
                                    "sha256": digest(source_path),
                                    "captured_at_utc": source["started_at_utc"],
                                    "input_sha256": source["input_sha256"]}
        for case in source["cases"]:
            if "expected_marker_id" not in case:
                continue
            path = (REPO / case["image"]).resolve()
            path.relative_to(REPO)
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise FileNotFoundError(path)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            original_hash = digest(path)
            target = case["expected_marker_id"]
            view = {"group": group, "name": case["name"], "target": target,
                    "distance_m": case["approach_distance_m"],
                    "original_image": path.relative_to(REPO).as_posix(),
                    "image": f"inputs/{group}/{path.name}", "image_sha256": original_hash,
                    "image_size_px": [image.shape[1], image.shape[0]],
                    "historic_ids": case["detected_ids"],
                    "native_default": detect(gray, dictionary), "results": {}}
            for rate in RATES:
                params = cv2.aruco.DetectorParameters_create()
                params.minMarkerDistanceRate = rate
                changed = {k for k, v in parameters_dict(params).items() if defaults[k] != v}
                assert changed == (set() if rate == 0.05 else {"minMarkerDistanceRate"})
                result = detect(gray, dictionary, params)
                result["target_detected"] = target in result["ids"]
                result["target_count"] = result["ids"].count(target)
                result["duplicate_ids"] = {str(k): v for k, v in Counter(result["ids"]).items() if v > 1}
                view["results"][str(rate)] = result
            view["baseline_matches_history"] = sorted(view["results"]["0.05"]["ids"]) == sorted(view["historic_ids"])
            view["explicit_matches_native"] = view["results"]["0.05"]["ids"] == view["native_default"]["ids"]
            view["target_changed_at_002"] = (view["results"]["0.05"]["target_detected"] != view["results"]["0.02"]["target_detected"])
            report["views"].append(view)
            copies.append((path, output / view["image"], original_hash))
    summary = {"images": len(report["views"]), "evaluations": len(report["views"]) * len(RATES),
               "baseline_matches_history": sum(v["baseline_matches_history"] for v in report["views"]),
               "explicit_matches_native": sum(v["explicit_matches_native"] for v in report["views"]),
               "groups": {}, "changed_at_002": []}
    for group in REPORTS:
        views = [v for v in report["views"] if v["group"] == group]
        summary["groups"][group] = {"images": len(views), "rates": {str(rate): {
            "target_detected": sum(v["results"][str(rate)]["target_detected"] for v in views),
            "target_duplicate_views": sum(v["results"][str(rate)]["target_count"] > 1 for v in views),
            "any_duplicate_views": sum(bool(v["results"][str(rate)]["duplicate_ids"]) for v in views),
            "detected_distances_m": {str(target): sorted({v["distance_m"] for v in views
                if v["target"] == target and v["results"][str(rate)]["target_detected"]}, reverse=True)
                for target in sorted({v["target"] for v in views})},
        } for rate in RATES}}
    summary["changed_at_002"] = [{"group": v["group"], "name": v["name"], "target": v["target"],
        "distance_m": v["distance_m"], "before": v["results"]["0.05"]["ids"],
        "after": v["results"]["0.02"]["ids"]} for v in report["views"] if v["target_changed_at_002"]]
    report["summary"] = summary
    output.mkdir(parents=True, exist_ok=False)
    for src, dest, expected in copies:
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dest)
        assert digest(src) == expected == digest(dest)
    for group, filename in REPORTS.items():
        shutil.copyfile(SOURCE / filename, output / f"{group}_original_report.json")
    (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if summary["baseline_matches_history"] != summary["images"] or summary["explicit_matches_native"] != summary["images"]:
        raise RuntimeError("기본값 재현 불일치: 새 결과를 과거 계수만의 효과로 단정하지 마세요.")


if __name__ == "__main__":
    main()
