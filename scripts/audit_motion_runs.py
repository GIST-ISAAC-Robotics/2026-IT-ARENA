#!/usr/bin/env python3
"""보정 주행 보고서와 당시 소스 ZIP을 대조합니다. 기존 결과는 변경하지 않습니다."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    if not root.is_relative_to(REPO / "artifacts"):
        raise ValueError("artifacts 내부 경로만 허용합니다.")
    output = root / "run_integrity_audit.json"
    if output.exists():
        raise FileExistsError(output)
    results = []
    for path in sorted(root.rglob("report.json")):
        report = json.loads(path.read_text())
        if not report.get("input_sha256"):
            continue
        archive_path = path.parent / "source_snapshot.zip"
        flat = not archive_path.exists()
        if flat:
            archive_path = path.parent / "source_snapshot_flat.zip"
        bad = []
        if archive_path.exists():
            with zipfile.ZipFile(archive_path) as archive:
                for relative, expected in report["input_sha256"].items():
                    member = Path(relative).name if flat else relative
                    if member not in archive.namelist() or hashlib.sha256(archive.read(member)).hexdigest() != expected:
                        bad.append(relative)
        else:
            bad.append("missing_source_archive")
        results.append({"report": str(path.relative_to(root)), "passed": report.get("passed"),
            "measurement_completed": report.get("measurement_completed"),
            "shutdown_clean": report.get("shutdown_clean"), "source_count": len(report["input_sha256"]),
            "archive_matches_recorded_hashes": not bad, "mismatches": bad})
    result = {"runs": results, "run_count": len(results),
              "all_source_archives_verified": bool(results) and all(r["archive_matches_recorded_hashes"] for r in results),
              "all_shutdown_clean": bool(results) and all(r["shutdown_clean"] for r in results),
              "scope": "archived run inputs, not current working-tree identity; preserves failed performance runs"}
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["all_source_archives_verified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
