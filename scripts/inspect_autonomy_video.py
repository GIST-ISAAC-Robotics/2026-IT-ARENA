#!/usr/bin/env python3
"""주행 MP4를 끝까지 디코드하고 시간대별 실제 프레임을 보존합니다."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    video, output = args.video.resolve(), args.output.resolve()
    if not video.is_relative_to(REPO / "artifacts") or not output.is_relative_to(REPO / "artifacts"):
        raise ValueError("영상과 검증 출력은 프로젝트 artifacts 안에 둡니다.")
    output.mkdir(parents=True, exist_ok=False)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError("영상 디코더를 열지 못했습니다.")
    declared = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    positions = sorted(set(round((declared-1)*f) for f in (0, .25, .50, .75, .95, 1)))
    count, frames, dimensions = 0, [], set()
    while True:
        valid, frame = capture.read()
        if not valid:
            break
        dimensions.add((frame.shape[1], frame.shape[0]))
        if count in positions:
            name = f"frame_{count:05d}.jpg"
            if not cv2.imwrite(str(output / name), frame):
                raise RuntimeError("검증 프레임 저장 실패")
            frames.append({"file": name, "frame_index": count, "video_time_s": count/fps})
        count += 1
    capture.release()
    report = {
        "source": str(video.relative_to(REPO)),
        "sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
        "declared_frames": declared, "decoded_frames": count, "fps": fps,
        "duration_s": count/fps, "dimensions": sorted(dimensions),
        "all_frames_decoded": count == declared and count > 0,
        "frames": frames,
        "scope": "video decoding only; driving completion is in the source run report",
    }
    (output / "video_inspection.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["all_frames_decoded"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
