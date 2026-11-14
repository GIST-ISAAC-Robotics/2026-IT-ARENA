"""저장 자세 재렌더 프레임을 1.0배속 30 fps 1920×1080 H.264 MP4로 합성하고 편집표·포스터를 만든다.

- 렌더 카메라는 시뮬레이션 1 ms 단계에서 약 30 Hz로 찍히므로 프레임 간격이 31–34 ms로 흔들린다.
  출력 프레임 k는 원 시각 t0 + k/30 s에 **가장 가까운 렌더 프레임**을 쓴다(보간·배속 없음, 오차 통계 기록).
- 오버레이: 제목·출처 표기·sim t·기록 사건 자막·(선택) 신호 기록 상태·신호등 근접 보조 화면·색 범례.
  사건 자막 문구와 시각은 clips.json(원 보고서·원시 기록에서 옮긴 값)과 render meta의 신호 단계만 사용한다.

사용(Windows 미디어 venv): python compose_clip.py --clip <key>
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

TOOLS = Path(__file__).resolve().parent
VIDEO_DIR = TOOLS.parent
REPO = TOOLS.parents[3]
BASE = REPO / "build" / "competition_recordings_20261115"
VAL_OUT = REPO / "artifacts" / "validation" / "2026-11-15" / "video_recording_v1"
W, H, FPS = 1920, 1080, 30
DT = 1.0 / FPS
FONT = "C:/Windows/Fonts/malgun.ttf"
FONT_BOLD = "C:/Windows/Fonts/malgunbd.ttf"
MAX_PICK_ERR_S = 0.020
SOURCE_NAMES = ("sources.zip", "execution_sources.zip", "source_snapshot.zip", "registered_untracked_sources.zip",
                "before.zip", "registered.json", "git_state_registered.txt", "config.json", "prepared.json",
                "report.json")
STAGE_LABEL = {"green_probe": "시험 점등(초록)", "off": "소등", "red1": "적색 1/4 점등", "red2": "적색 2/4 점등",
               "red3": "적색 3/4 점등", "red4": "적색 4/4 점등", "green": "초록 점등"}


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, size)


F_TITLE, F_SUB, F_TAG, F_SMALL, F_CAP = font(44, True), font(28), font(25), font(26), font(31, True)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rel(path):
    return str(Path(path).resolve().relative_to(REPO)).replace("\\", "/")


def load_clip(key):
    spec = json.load(open(TOOLS / "clips.json", encoding="utf-8"))
    return next(c for c in spec["clips"] if c["key"] == key)


def frame_list(folder):
    files = sorted(folder.glob("f_*.jpg"))
    return np.array([int(f.stem[2:]) for f in files], dtype=np.int64), files


def box(draw, xy, lines, anchor="la", pad=12, gap=6):
    """여러 줄(글꼴, 문구, 색)을 반투명 상자 하나에 그린다."""
    x, y = xy
    sizes = [draw.textbbox((0, 0), text, font=fnt) for fnt, text, _ in lines]
    widths = [b[2] - b[0] for b in sizes]
    heights = [b[3] - b[1] for b in sizes]
    bw = max(widths) + 2 * pad
    bh = sum(heights) + gap * (len(lines) - 1) + 2 * pad
    left = x if anchor[0] == "l" else (x - bw if anchor[0] == "r" else x - bw // 2)
    top = y if anchor[1] == "a" else y - bh
    draw.rounded_rectangle((left, top, left + bw, top + bh), radius=8, fill=(0, 0, 0, 155))
    cy = top + pad
    for (fnt, text, color), b, hgt in zip(lines, sizes, heights):
        draw.text((left + pad - b[0], cy - b[1]), text, font=fnt, fill=color)
        cy += hgt + gap
    return left, top, left + bw, top + bh


def signal_state(stages, t):
    before = [s for s in stages if s["applied_sim_s"] <= t + 1e-9]
    return before[-1]["stage"] if before else None


def overlay(frame, clip, meta, t, inset):
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    white, grey, yellow = (255, 255, 255, 255), (215, 215, 215, 255), (255, 226, 120, 255)
    box(d, (32, 30), [(F_TITLE, clip["title"], white), (F_SUB, clip["subtitle"], grey)])
    _, _, _, tag_bottom = box(d, (W - 32, 30), [(F_TAG, "Gazebo 시뮬레이션 · 저장 자세 재렌더 · 1.0×", white),
                                                (F_TAG, "실차 영상 아님 · " + clip["category"], grey)], anchor="ra")
    if inset is not None:
        iw, ih = 576, 324
        x0, y0 = W - 32 - iw, tag_bottom + 14
        label = box(d, (x0 + iw, y0), [(F_TAG, "신호등 근접 보조 화면 (같은 시각 재렌더)", white)], anchor="ra", pad=8)
        y1 = label[3] + 4
        layer.paste(Image.fromarray(inset).convert("RGBA").resize((iw, ih), Image.LANCZOS), (x0, y1))
        d.rectangle((x0 - 2, y1 - 2, x0 + iw + 1, y1 + ih + 1), outline=(255, 255, 255, 230), width=2)
    green = clip.get("green_s")
    rows = []
    if clip.get("signal_hud_until") and t <= clip["signal_hud_until"]:
        stage = signal_state(meta["signal_stages"], t)
        rows.append((F_SMALL, "신호 기록: " + STAGE_LABEL.get(stage, "점등 기록 전"), yellow))
    timing = f"sim t = {t:6.2f} s"
    if green is not None:
        timing += f"   GREEN + {t - green:5.2f} s" if t >= green else f"   GREEN까지 {green - t:5.2f} s"
    rows.append((F_SMALL, timing, white))
    box(d, (32, H - 30), rows, anchor="ld")
    legend = "파랑 = 자차" + ("    빨강 = 상대차(평가용 각본)" if clip.get("actor") else "")
    box(d, (W - 32, H - 30), [(F_SMALL, legend, white)], anchor="rd")
    active = [e for e in clip["events"] if e["t"] - 1e-6 <= t < e["t"] + e.get("dur", 2.6)]
    if active:
        box(d, (W // 2, H - 118), [(F_CAP, active[-1]["text"], white)], anchor="ma")
    return np.asarray(Image.alpha_composite(img, layer).convert("RGB"))


def poster(frame, clip, path, inset=None):
    img = Image.fromarray(frame).convert("RGBA").resize((1280, 720), Image.LANCZOS)
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    for i in range(230):
        d.line((0, 720 - i, 1280, 720 - i), fill=(0, 0, 0, int(205 * (1 - i / 230) ** 0.8)))
    d.text((44, 720 - 178), clip["title"], font=font(clip.get("poster_title_size", 62), True), fill=(255, 255, 255, 255))
    d.text((46, 720 - 98), clip["subtitle"], font=font(31), fill=(230, 230, 230, 255))
    d.text((46, 720 - 52), "Gazebo 시뮬레이션 · 저장 자세 재렌더 · " + clip["category"], font=font(23),
           fill=(255, 226, 120, 255))
    if inset is not None:
        iw, ih = 384, 216
        x0, y0 = 1280 - 24 - iw, 24
        layer.paste(Image.fromarray(inset).convert("RGBA").resize((iw, ih), Image.LANCZOS), (x0, y0))
        d.rectangle((x0 - 2, y0 - 2, x0 + iw + 1, y0 + ih + 1), outline=(255, 255, 255, 230), width=2)
        d.text((x0 + 6, y0 + ih + 6), "신호등 근접 보조 화면(같은 시각)", font=font(20), fill=(255, 255, 255, 255),
               stroke_width=2, stroke_fill=(0, 0, 0, 255))
    Image.alpha_composite(img, layer).convert("RGB").save(path, quality=90, optimize=True)


def source_files(run_dir):
    found, folder = [], run_dir
    for _ in range(3):
        for name in SOURCE_NAMES:
            p = folder / name
            if p.exists():
                found.append(dict(path=rel(p), sha256=sha256(p), bytes=p.stat().st_size))
        folder = folder.parent
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clip", required=True)
    parser.add_argument("--poster-only", action="store_true", help="MP4는 그대로 두고 포스터만 다시 만든다")
    args = parser.parse_args()
    clip = load_clip(args.clip)
    meta = json.load(open(BASE / "logs" / clip["key"] / "meta.json", encoding="utf-8"))
    offset = meta["render_tau_to_sim_offset_s"]
    ns, files = frame_list(BASE / "frames" / clip["key"] / "frames_main")
    times = ns * 1e-9 + offset
    inset_dir = BASE / "frames" / clip["key"] / "frames_signal"
    ins_ns, ins_files = frame_list(inset_dir) if inset_dir.exists() else (np.array([], dtype=np.int64), [])
    t0, t1 = clip["t0"], clip["t1"]
    n_out = int(np.floor((t1 - t0) / DT + 1e-6)) + 1
    targets = t0 + np.arange(n_out) * DT
    if times[0] > t0 + MAX_PICK_ERR_S or times[-1] < targets[-1] - MAX_PICK_ERR_S:
        raise SystemExit(f"rendered range {times[0]:.3f}-{times[-1]:.3f} does not cover {t0}-{targets[-1]:.3f}")
    picks = np.array([int(np.argmin(np.abs(times - x))) for x in targets])
    err = times[picks] - targets
    if np.max(np.abs(err)) > MAX_PICK_ERR_S:
        raise SystemExit(f"nearest frame error {np.max(np.abs(err)):.4f} s exceeds {MAX_PICK_ERR_S}")
    repeats = int(np.sum(np.diff(picks) == 0))
    skips = int(np.sum(np.diff(picks) > 1))
    render_gaps = np.diff(times)

    VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    out_mp4 = VIDEO_DIR / f"{clip['file']}.mp4"
    posters = VIDEO_DIR / "posters"
    posters.mkdir(exist_ok=True)
    poster_path = posters / f"{clip['file']}_poster.jpg"
    pidx = int(picks[np.argmax(times[picks] >= clip["poster_t"])])
    with Image.open(files[pidx]) as im:
        poster_frame = np.asarray(im.convert("RGB"))
    poster_inset = None
    if clip.get("poster_inset") and len(ins_ns):
        with Image.open(ins_files[int(np.argmin(np.abs(ins_ns - ns[pidx])))]) as im:
            poster_inset = np.asarray(im.convert("RGB"))
    poster(poster_frame, clip, poster_path, poster_inset)
    poster_info = dict(poster=rel(poster_path), poster_sha256=sha256(poster_path),
                       poster_source_sim_s=round(float(times[pidx]), 3))
    edit_path = VAL_OUT / "edit" / f"{clip['file']}.edit.json"
    if args.poster_only:
        edit = json.load(open(edit_path, encoding="utf-8"))
        edit.update(poster_info)
        edit_path.write_text(json.dumps(edit, ensure_ascii=False, indent=2) + chr(10), encoding="utf-8", newline=chr(10))
        print(json.dumps(poster_info, ensure_ascii=False))
        return
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    cmd = [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-preset", "slow", "-crf", "20",
           "-pix_fmt", "yuv420p", "-profile:v", "high", "-movflags", "+faststart", "-r", str(FPS), str(out_mp4)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for k, (target, idx) in enumerate(zip(targets, picks)):
        t = float(times[idx])
        with Image.open(files[idx]) as im:
            frame = np.asarray(im.convert("RGB"))
        if frame.shape[:2] != (H, W):
            raise SystemExit(f"unexpected frame size {frame.shape}")
        inset = None
        if len(ins_ns) and clip.get("inset_until") and t <= clip["inset_until"]:
            j = int(np.argmin(np.abs(ins_ns - ns[idx])))
            if ins_ns[j] != ns[idx]:
                raise SystemExit("inset frame stamp mismatch")
            with Image.open(ins_files[j]) as im:
                inset = np.asarray(im.convert("RGB"))
        proc.stdin.write(overlay(frame, clip, meta, t, inset).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise SystemExit("ffmpeg failed")
    render_result = (BASE / "logs" / clip["key"] / "render_result.txt").read_text(encoding="utf-8").strip()
    tool_files = ["build_scene.py", "render_clip.sh", "render_all.sh", "convert_frames.py", "compose_clip.py",
                  "qa_clip.py", "clips.json", "render_plugin/pose_replay_system.cpp", "render_plugin/CMakeLists.txt"]
    events = []
    for e in clip["events"]:
        inside = t0 - 1e-6 <= e["t"] <= targets[-1] + 1e-6
        events.append(dict(e, video_s=round(e["t"] - t0, 3) if inside else None))
    for s in meta["signal_stages"]:
        if t0 <= s["applied_sim_s"] <= targets[-1]:
            events.append(dict(t=s["applied_sim_s"], text="신호 기록 " + STAGE_LABEL.get(s["stage"], s["stage"]),
                               video_s=round(s["applied_sim_s"] - t0, 3), source="signal_sequence.jsonl"))
    events.sort(key=lambda e: e["t"])
    run_dir = REPO / "artifacts" / "validation" / clip["run_dir"]
    result = dict(
        clip=clip["key"], output=rel(out_mp4), sha256=sha256(out_mp4), bytes=out_mp4.stat().st_size,
        **poster_info,
        kind="saved-pose re-render (Gazebo ogre2, physics/control not re-executed); not a new simulation run",
        title=clip["title"], subtitle=clip["subtitle"], category=clip["category"],
        facts=clip["facts"], conditions=clip["conditions"], report=clip["report"],
        source_run=rel(run_dir), source_files=source_files(run_dir), render_inputs_sha256=meta["input_sha256"],
        external_textures=meta.get("external_textures"),
        video=dict(width=W, height=H, fps=FPS, frames=n_out, duration_s=round(n_out * DT, 3), playback_rate=1.0,
                   codec="libx264 high crf20 preset slow yuv420p +faststart (imageio-ffmpeg static build)", audio=None),
        sim_window=dict(first_frame_target_s=t0, last_frame_target_s=round(float(targets[-1]), 3),
                        first_frame_actual_s=round(float(times[picks[0]]), 3), last_frame_actual_s=round(float(times[picks[-1]]), 3)),
        frame_mapping=dict(rule="output frame k uses the rendered frame nearest to t0 + k/30 s",
                           max_abs_error_ms=round(float(np.max(np.abs(err))) * 1e3, 2),
                           mean_abs_error_ms=round(float(np.mean(np.abs(err))) * 1e3, 2),
                           repeated_source_frames=repeats, skipped_source_frames=skips,
                           render_gap_ms=[round(float(render_gaps.min()) * 1e3, 1), round(float(render_gaps.max()) * 1e3, 1)],
                           rendered_frames=len(files)),
        render=dict(result=render_result, camera=meta["camera"], camera_qa=meta["camera_qa"],
                    signal_inset=meta.get("signal_inset"), inset_until_s=clip.get("inset_until"),
                    physics_step_s=meta["physics_step_s"], actor=meta.get("actor"),
                    plugin_sha256=sha256(BASE / "plugin" / "librecording-pose-replay.so"),
                    tools_sha256={f: sha256(TOOLS / f) for f in tool_files if (TOOLS / f).exists()}),
        time_note="영상 1초 = 원 시뮬레이션 1초(1.0배속). 렌더 wall 시간은 원 실행의 실제 처리율이나 실차 시간과 무관하다.",
        events=events)
    edit_path.parent.mkdir(parents=True, exist_ok=True)
    edit_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                                                        encoding="utf-8", newline="\n")
    print(json.dumps(dict(clip=clip["key"], frames=n_out, sha256=result["sha256"], bytes=result["bytes"],
                          mapping=result["frame_mapping"]), ensure_ascii=False))


if __name__ == "__main__":
    main()
