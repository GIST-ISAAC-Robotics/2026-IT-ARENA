#!/usr/bin/env python3
"""완료된 주행 기록의 자식 프로세스 종료·시간·조향 출처를 재감사합니다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from lidar_shutdown import audit_shutdown

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    folder = args.run.resolve()
    if not folder.is_relative_to(REPO / "artifacts"):
        raise ValueError("주행 기록은 프로젝트 artifacts 안에서 선택합니다.")
    output = folder / "recording_audit.json"
    if output.exists():
        raise FileExistsError("기존 재감사는 덮어쓰지 않습니다.")
    report = json.loads((folder / "report.json").read_text(encoding="utf-8"))
    trace = json.loads((folder / "trace.json").read_text(encoding="utf-8"))
    shutdown = audit_shutdown((folder / "simulation.log").read_text(encoding="utf-8"),
                              report["process_exit_codes"][0])
    active = [row for row in trace if row["autonomy"].get("started")]
    # 상태 토픽의 반복 수신 표본을 원시 20 Hz 조향 실행 횟수로 해석하지 않습니다.
    unique = {row["autonomy"]["sim_time_s"]: row for row in active}
    sources = Counter(row["autonomy"].get("steering_source", "lidar_wall_follow")
                      for row in unique.values())
    changed = []
    for relative, expected in report["input_sha256"].items():
        path = (REPO / relative).resolve()
        if not path.is_relative_to(REPO):
            raise ValueError("기록 안의 소스 경로가 프로젝트 밖을 가리킵니다.")
        current = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        if current != expected:
            changed.append({"path": relative, "at_run_sha256": expected,
                            "current_sha256": current})
    span = active[-1]["time"] - active[0]["time"] if len(active) > 1 else None
    result = {
        "source_run": str(folder.relative_to(REPO)),
        "source_report_passed": report["passed"],
        "shutdown": shutdown,
        "active_trace_span_sim_s": span,
        "video_duration_s": report.get("video", {}).get("duration_s"),
        "unique_status_samples": len(unique),
        "steering_sources": dict(sources),
        "source_changes_since_run": changed,
        "interpretation": "read-only post-run audit; source changes do not retroactively change recorded results",
    }
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if shutdown["clean"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
