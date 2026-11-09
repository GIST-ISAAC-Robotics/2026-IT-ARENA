"""최종 MP4를 실제로 끝까지 디코드해 형식·프레임·정지/검은 화면을 검사하고 contact sheet를 만든다.

사용: python qa_showcase.py --dir <feature_showcase 폴더>
출력: qa_report.json, contact_sheet.png
"""
import argparse
import json
import struct
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

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


def pick_times(table):
    picks = []
    for seg in table["segments"]:
        start, end = seg["video_start_s"], seg["video_end_s"]
        picks.append((start + 0.3, seg, "시작"))
        chosen = [e for e in seg["events"] if e["in_clip"] and "녹색" not in e["what"] and "진행" not in e["what"]]
        if len(chosen) > 4:
            idx = np.linspace(0, len(chosen) - 1, 4).round().astype(int)
            chosen = [chosen[i] for i in sorted(set(idx))]
        for e in chosen:
            picks.append((e["video_s"], seg, e["what"].split(",")[0]))
        picks.append((end - 0.3, seg, "끝"))
    return picks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=Path, required=True)
    args = parser.parse_args()
    mp4 = args.dir / "feature_showcase.mp4"
    table = json.load(open(args.dir / "edit_table.json", encoding="utf-8"))
    container = av.open(str(mp4))
    stream = container.streams.video[0]
    info = dict(codec=stream.codec_context.name, profile=stream.codec_context.profile,
                pix_fmt=stream.codec_context.pix_fmt, width=stream.codec_context.width,
                height=stream.codec_context.height, average_rate=str(stream.average_rate),
                time_base=str(stream.time_base), container_duration_s=container.duration / 1e6 if container.duration else None,
                audio_streams=len(container.streams.audio))
    fps = float(stream.average_rate)
    picks = pick_times(table)
    wanted = {int(round(t * fps)): (t, seg, label) for t, seg, label in picks}
    thumbs = {}
    prev, decoded, pts = None, 0, []
    same_run, longest_same, longest_same_at = 0, 0, None
    dark = []
    for frame in container.decode(video=0):
        arr = frame.to_ndarray(format="rgb24")
        pts.append(float(frame.pts * stream.time_base))
        if prev is not None:
            diff = float(np.mean(np.abs(arr.astype(np.int16) - prev.astype(np.int16))))
            same_run = same_run + 1 if diff < 0.3 else 0
            if same_run > longest_same:
                longest_same, longest_same_at = same_run, decoded
        if float(arr.mean()) < 15:
            dark.append(decoded)
        if decoded in wanted:
            thumbs[decoded] = Image.fromarray(arr).resize((400, 225), Image.LANCZOS)
        prev = arr
        decoded += 1
    container.close()
    steps = np.diff(pts)
    report = dict(file=mp4.name, bytes=mp4.stat().st_size, stream=info, decoded_frames=decoded,
                  expected_frames=table["frames"], decoded_duration_s=round(decoded / fps, 3),
                  pts_step_min_s=float(steps.min()), pts_step_max_s=float(steps.max()),
                  top_level_boxes=top_level_boxes(mp4),
                  faststart=top_level_boxes(mp4).index("moov") < top_level_boxes(mp4).index("mdat"),
                  longest_identical_run_frames=longest_same + 1 if longest_same else 0,
                  longest_identical_run_s=round((longest_same + 1) / fps, 3) if longest_same else 0.0,
                  longest_identical_run_ends_at_s=round(longest_same_at / fps, 3) if longest_same_at else None,
                  dark_frames=len(dark),
                  definitions=dict(identical="mean absolute RGB difference < 0.3 against previous frame",
                                   dark="mean RGB < 15"))
    fnt, fnt_small = ImageFont.truetype(FONT, 17), ImageFont.truetype(FONT, 15)
    cols, tw, th, cap = 4, 400, 225, 48
    keys = sorted(thumbs)
    rows = (len(keys) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tw + (cols + 1) * 8, rows * (th + cap) + (rows + 1) * 8 + 40), (24, 24, 24))
    draw = ImageDraw.Draw(sheet)
    draw.text((10, 10), f"feature_showcase.mp4 contact sheet — {decoded} frames · {decoded / fps:.2f} s · 20 fps · 1.0× (영상시각 / 원 시뮬시각)",
              font=fnt, fill=(235, 235, 235))
    for n, key in enumerate(keys):
        t, seg, label = wanted[key]
        sim = seg["sim_first_frame_s"] + (key / fps - seg["video_start_s"])
        x = 8 + (n % cols) * (tw + 8)
        y = 48 + (n // cols) * (th + cap + 8)
        sheet.paste(thumbs[key], (x, y))
        draw.text((x, y + th + 3), f"{key / fps:6.2f}s | sim {sim:6.2f}s | {seg['key']}", font=fnt_small, fill=(230, 230, 230))
        draw.text((x, y + th + 23), label[:34], font=fnt_small, fill=(180, 200, 255))
    sheet.save(args.dir / "contact_sheet.png", optimize=True)
    report["contact_sheet_frames"] = [dict(video_s=round(k / fps, 2), segment=wanted[k][1]["key"], label=wanted[k][2]) for k in keys]
    (args.dir / "qa_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("decoded_frames", "expected_frames", "decoded_duration_s", "faststart",
                                             "longest_identical_run_s", "dark_frames")}, ensure_ascii=False))
    print(json.dumps(info))


if __name__ == "__main__":
    main()
