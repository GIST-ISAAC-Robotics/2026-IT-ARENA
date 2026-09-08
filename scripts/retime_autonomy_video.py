#!/usr/bin/env python3
"""수신이 빠진 프레임을 직전 영상으로 유지하여 시뮬레이션 1배 시간 MP4를 만듭니다."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]


def frame_schedule(stamps, fps=20.):
    stamps = np.asarray(stamps, dtype=float)
    if (len(stamps) < 2 or not np.all(np.isfinite(stamps)) or
            np.any(np.diff(stamps) <= 0) or not np.isfinite(fps) or fps <= 0):
        raise ValueError("단조 증가하는 영상 시각과 양수 FPS가 필요합니다.")
    ticks = np.arange(int(np.floor((stamps[-1] - stamps[0]) * fps + 1e-7)) + 1) / fps + stamps[0]
    return np.maximum(0, np.searchsorted(stamps, ticks + 1e-9, side="right") - 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    folder = args.run.resolve()
    if not folder.is_relative_to(REPO / "artifacts"):
        raise ValueError("artifacts 안의 실행만 처리합니다.")
    report = json.loads((folder / "report.json").read_text())
    source = REPO / report["video"]["path"]
    if not source.resolve().is_relative_to(folder):
        raise ValueError("원본 영상 경로가 실행 폴더 밖입니다.")
    output = folder / "lidar_third_person_simtime_1x.mp4"
    audit = folder / "video_timing_audit.json"
    if output.exists() or audit.exists():
        raise FileExistsError("기존 영상/감사는 덮어쓰지 않습니다.")
    stamps = json.loads((folder / "video_timestamps.json").read_text())
    schedule = frame_schedule(stamps)
    capture = cv2.VideoCapture(str(source))
    writer = None
    read_index, current = -1, None
    try:
        for index in schedule:
            while read_index < index:
                ok, current = capture.read()
                if not ok:
                    raise RuntimeError("원본 영상 프레임 부족")
                read_index += 1
            if writer is None:
                writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"avc1"), 20.,
                                         (current.shape[1], current.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError("H.264 인코더 실패")
            writer.write(current)
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    result = {"source": str(source.relative_to(REPO)), "output": str(output.relative_to(REPO)),
              "fps": 20., "source_frames": len(stamps), "output_frames": len(schedule),
              "simulation_span_s": stamps[-1] - stamps[0], "output_duration_s": len(schedule) / 20.,
              "max_source_gap_s": float(max(np.diff(stamps))),
              "held_frames": int(np.sum(np.diff(schedule) == 0)),
              "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
              "scope": "simulation-time 1x; missing images held, no synthesized motion or speed-up; not wall-clock RTF"}
    audit.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
