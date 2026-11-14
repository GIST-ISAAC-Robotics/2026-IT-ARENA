"""완성 MP4를 끝까지 디코드해 형식·프레임 수·PTS·검은 화면·정지 화면을 검사하고 대표 contact sheet를 만든다.

ffprobe 실행 파일이 없는 환경이므로 PyAV(libav) 스트림 정보와 `ffmpeg -i` 출력(stderr)을 함께 남긴다.
사용: python qa_clip.py --clip <key>
"""
import argparse
import json
import struct
import subprocess
from pathlib import Path

import av
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

TOOLS = Path(__file__).resolve().parent
VIDEO_DIR = TOOLS.parent
REPO = TOOLS.parents[3]
VAL_OUT = REPO / "artifacts" / "validation" / "2026-11-15" / "video_recording_v1"
FONT = "C:/Windows/Fonts/malgun.ttf"


def top_level_boxes(path):
    boxes = []
    with open(path, "rb") as handle:
        while True:
            header = handle.read(8)
            if len(header) < 8:
                break
            size, kind = struct.unpack(">I4s", header)
            if size == 1:
                size = struct.unpack(">Q", handle.read(8))[0]
                handle.seek(size - 16, 1)
            else:
                handle.seek(size - 8, 1)
            boxes.append(kind.decode("latin1"))
            if size == 0:
                break
    return boxes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clip", required=True)
    args = parser.parse_args()
    spec = json.load(open(TOOLS / "clips.json", encoding="utf-8"))
    clip = next(c for c in spec["clips"] if c["key"] == args.clip)
    edit = json.load(open(VAL_OUT / "edit" / f"{clip['file']}.edit.json", encoding="utf-8"))
    mp4 = VIDEO_DIR / f"{clip['file']}.mp4"
    probe = subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-hide_banner", "-i", str(mp4)],
                           capture_output=True, text=True, encoding="utf-8", errors="replace").stderr
    t0 = clip["t0"]
    picks = [(0.4, "시작")]
    for e in edit["events"]:
        if e.get("video_s") is not None and e.get("source") != "signal_sequence.jsonl":
            picks.append((e["video_s"] + 0.5, e["text"].split("·")[0].split("(")[0].strip()))
    if any(e.get("source") == "signal_sequence.jsonl" for e in edit["events"]):
        for e in edit["events"]:
            if e.get("source") == "signal_sequence.jsonl" and ("4/4" in e["text"] or "초록" in e["text"] or "1/4" in e["text"]):
                picks.append((e["video_s"] + 0.2, e["text"]))
    duration = edit["video"]["duration_s"]
    for frac in (0.25, 0.5, 0.75):
        picks.append((duration * frac, f"진행 {int(frac * 100)}%"))
    picks.append((clip["poster_t"] - t0, "포스터 시점"))
    picks.append((duration - 0.2, "끝"))
    container = av.open(str(mp4))
    stream = container.streams.video[0]
    cc = stream.codec_context
    info = dict(codec=cc.name, profile=cc.profile, pix_fmt=cc.pix_fmt, width=cc.width, height=cc.height,
                average_rate=str(stream.average_rate), time_base=str(stream.time_base),
                container_duration_s=container.duration / 1e6 if container.duration else None,
                audio_streams=len(container.streams.audio), bit_rate=container.bit_rate)
    fps = float(stream.average_rate)
    wanted = {}
    for t, label in picks:
        k = int(round(min(max(t, 0.0), duration - 1.0 / fps) * fps))
        wanted.setdefault(k, label)
    thumbs, pts = {}, []
    prev, decoded = None, 0
    same_run, longest_same, longest_end = 0, 0, None
    dark = 0
    for frame in container.decode(video=0):
        arr = frame.to_ndarray(format="rgb24")
        pts.append(float(frame.pts * stream.time_base))
        if prev is not None:
            diff = float(np.mean(np.abs(arr[::4, ::4].astype(np.int16) - prev[::4, ::4].astype(np.int16))))
            same_run = same_run + 1 if diff < 0.3 else 0
            if same_run > longest_same:
                longest_same, longest_end = same_run, decoded
        if float(arr[::4, ::4].mean()) < 15:
            dark += 1
        if decoded in wanted:
            thumbs[decoded] = Image.fromarray(arr).resize((480, 270), Image.LANCZOS)
        prev = arr
        decoded += 1
    container.close()
    steps = np.diff(pts)
    boxes = top_level_boxes(mp4)
    report = dict(file=edit["output"], sha256=edit["sha256"], bytes=mp4.stat().st_size, stream=info,
                  decoded_frames=decoded, expected_frames=edit["video"]["frames"],
                  frames_match=decoded == edit["video"]["frames"], decoded_duration_s=round(decoded / fps, 3),
                  pts_step_min_s=round(float(steps.min()), 6), pts_step_max_s=round(float(steps.max()), 6),
                  top_level_boxes=boxes, faststart=boxes.index("moov") < boxes.index("mdat"),
                  longest_identical_run_s=round((longest_same + 1) / fps, 3) if longest_same else 0.0,
                  longest_identical_run_ends_at_video_s=round(longest_end / fps, 3) if longest_end else None,
                  dark_frames=dark,
                  definitions=dict(identical="mean abs RGB diff < 0.3 vs previous frame (4x subsampled)",
                                   dark="mean RGB < 15 (4x subsampled)"),
                  ffmpeg_i_stderr=probe.strip().splitlines()[-6:])
    fnt, small = ImageFont.truetype(FONT, 20), ImageFont.truetype(FONT, 17)
    keys = sorted(thumbs)
    cols = 4
    rows = (len(keys) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * 480 + (cols + 1) * 8, rows * (270 + 52) + (rows + 1) * 8 + 46), (24, 24, 24))
    d = ImageDraw.Draw(sheet)
    d.text((10, 10), f"{mp4.name} — {decoded} frames · {decoded / fps:.2f} s · {fps:g} fps · 1.0× (영상 시각 / 원 sim 시각)",
           font=fnt, fill=(235, 235, 235))
    for n, k in enumerate(keys):
        x = 8 + (n % cols) * (480 + 8)
        y = 46 + (n // cols) * (270 + 52 + 8)
        sheet.paste(thumbs[k], (x, y))
        d.text((x, y + 273), f"{k / fps:6.2f} s | sim {t0 + k / fps:6.2f} s", font=small, fill=(230, 230, 230))
        d.text((x, y + 296), wanted[k][:30], font=small, fill=(180, 200, 255))
    sheets = VAL_OUT / "contact_sheets"
    sheets.mkdir(parents=True, exist_ok=True)
    sheet_path = sheets / f"{clip['file']}_contact_sheet.jpg"
    sheet.save(sheet_path, quality=88, optimize=True)
    report["contact_sheet"] = str(sheet_path.relative_to(REPO)).replace("\\", "/")
    report["contact_sheet_frames"] = [dict(video_s=round(k / fps, 2), sim_s=round(t0 + k / fps, 2), label=wanted[k]) for k in keys]
    qa_dir = VAL_OUT / "qa"
    qa_dir.mkdir(parents=True, exist_ok=True)
    (qa_dir / f"{clip['file']}.qa.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                    encoding="utf-8", newline="\n")
    print(json.dumps({k: report[k] for k in ("decoded_frames", "expected_frames", "frames_match", "faststart",
                                             "longest_identical_run_s", "dark_frames", "pts_step_min_s", "pts_step_max_s")},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
