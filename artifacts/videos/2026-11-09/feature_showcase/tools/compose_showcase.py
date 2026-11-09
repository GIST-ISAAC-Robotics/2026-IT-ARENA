"""기능 소개 영상 편집: 기존 r25 3인칭 원본 + 교통 기록 재렌더 장면을 1배속 컷편집으로 잇는다.

- 모든 장면은 20 fps, 프레임 1장 = 시뮬레이션 0.05 s(1.0배속). 프레임 복제·보간·배속 없음.
- 오버레이는 장면 이름·출처 표기·시뮬레이션 시각·색 구분 한 줄만 그린다.
- 편집표(edit_table.json/.md)와 사건 시각은 원시 기록(trajectory/ego/actor 자세)에서 계산한다.

사용(Windows 미디어 venv):
  python compose_showcase.py --work C:/Users/Jinhyeong/arena_media_work --out <feature_showcase 폴더>
"""
import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import imageio_ffmpeg

REPO = Path(__file__).resolve().parents[5]
VAL = REPO / "artifacts" / "validation"
W, H, FPS = 1280, 720, 20
DT = 1.0 / FPS
FONT = "C:/Windows/Fonts/malgun.ttf"
FONT_BOLD = "C:/Windows/Fonts/malgunbd.ttf"

SEGMENTS = [
    dict(key="solo_r25", kind="video", title="① 단독 주행 · 거리 한 랩", run="r25",
         run_dir="2026-11-08/solo_gap_envelope_connected_v3/run1",
         report="2026-11-08/solo_gap_envelope_connected_v3/README.md",
         video="local_pursuit_third_person_drive.mp4", cut=(8.25, 37.5), actor=False),
    dict(key="slow_030", kind="render", title="② 저속 0.30 m/s 앞차 추월", run="r13",
         run_dir="2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1",
         report="2026-11-07/traffic_piecewise_v1/README.md", cut=(7.70, 18.40), actor=True),
    dict(key="medium_050_recessed", kind="render", title="③ 중속 0.50 m/s 비정형 앞차 추월", run="오목 외형 1회",
         run_dir="2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1",
         report="2026-11-08/medium_recessed_pass_v1/README.md", cut=(8.40, 17.40), actor=True),
    dict(key="yield_back", kind="render", title="④ 앞차 가속 → 양보 후 뒤로 복귀", run="r9",
         run_dir="2026-11-07/return_guide_front_v1/straight_moving_lead/run2",
         report="2026-11-07/return_guide_front_v1/README.md", cut=(9.00, 17.40), actor=True),
    dict(key="static_bypass", kind="render", title="⑤ 정지차 우회 후 본선 복귀", run="r18",
         run_dir="2026-11-08/rear_face_motion_connected_v1/persistent_block/run1",
         report="2026-11-08/rear_face_motion_connected_v1/README.md", cut=(8.70, 19.20), actor=True),
    dict(key="stop_restart", kind="render", title="⑥ 앞차 정지·재출발 → 뒤로 합류", run="r18 재출발",
         run_dir="2026-11-08/rear_face_restart_v1/moving_stop_restart/run1",
         report="2026-11-08/rear_face_restart_v1/README.md", cut=(10.20, 23.40), actor=True),
    dict(key="overtaken", kind="render", title="⑦ 빠른 뒤차에게 추월당함", run="r24",
         run_dir="2026-11-08/rear_overtake_measured_v1/left_rear_overtakes/run1",
         report="2026-11-08/rear_overtake_measured_v1/README.md", cut=(8.60, 14.40), actor=True),
]


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, size)


F_TITLE, F_SMALL = font(26, True), font(18)


def label_box(draw, xy, text, fnt, anchor="la"):
    x, y = xy
    box = draw.textbbox((x, y), text, font=fnt, anchor=anchor)
    pad = 7
    draw.rounded_rectangle((box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad), radius=6, fill=(0, 0, 0, 150))
    draw.text((x, y), text, font=fnt, fill=(255, 255, 255, 255), anchor=anchor)
    return box


def overlay(frame, seg, sim_t):
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    # r25 원본 영상에는 상단 왼쪽 정보 패널이 새겨져 있으므로 그 아래에 장면 이름을 둔다.
    label_box(draw, (22, 168 if seg["kind"] == "video" else 22), seg["title"], F_TITLE)
    tag = ("Gazebo 시뮬레이션 원본 녹화" if seg["kind"] == "video" else "Gazebo 시뮬레이션 · 기록 자세 재렌더")
    label_box(draw, (W - 22, 24), f"{tag} · {seg['run']} · 1.0×", F_SMALL, anchor="ra")
    label_box(draw, (22, H - 22), f"sim t = {sim_t:6.2f} s", F_SMALL, anchor="ld")
    legend = "파랑 = 자차" + ("   빨강 = 상대차(평가용 각본)" if seg["actor"] else "")
    label_box(draw, (W - 22, H - 22), legend, F_SMALL, anchor="rd")
    return np.asarray(Image.alpha_composite(img, layer).convert("RGB"))


def load_jsonl(path):
    return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]


def video_frames(seg):
    run = VAL / seg["run_dir"]
    stamps = json.load(open(run / "video_timestamps.json", encoding="utf-8"))
    container = av.open(str(run / seg["video"]))
    frames = [f.to_ndarray(format="rgb24") for f in container.decode(video=0)]
    container.close()
    if len(frames) != len(stamps):
        raise ValueError(f"decoded {len(frames)} != stamps {len(stamps)}")
    out = []
    for stamp, frame in zip(stamps, frames):
        if seg["cut"][0] - 1e-6 <= stamp <= seg["cut"][1] + 1e-6:
            img = Image.fromarray(frame).resize((W, H), Image.LANCZOS)
            out.append((stamp, np.asarray(img)))
    return out, dict(source_frames=len(frames), source_size=list(frames[0].shape[1::-1]),
                     scaled_to=[W, H], scale_filter="PIL LANCZOS")


def render_frames(seg, work):
    folder = work / "clips" / seg["key"]
    meta = json.load(open(folder / "meta.json", encoding="utf-8"))
    offset = meta["render_tau_to_sim_offset_s"]
    out = []
    for path in sorted(folder.glob("f_*.png")):
        tau = int(path.stem[2:]) * 1e-9
        t = round(tau + offset, 6)
        if seg["cut"][0] - 1e-6 <= t <= seg["cut"][1] + 1e-6:
            with Image.open(path) as im:
                out.append((t, np.asarray(im.convert("RGB"))))
    return out, dict(render_meta=meta)


def scene_events(seg):
    """원시 기록에서 사건 시각을 계산한다(평가 정답 자세는 사후 설명에만 사용)."""
    run = VAL / seg["run_dir"]
    traj = load_jsonl(run / "trajectory.jsonl")
    for r in traj:
        r.setdefault("t", r.get("time"))
    events = []
    if not seg["actor"]:
        summary = json.load(open(run.parent / "live_summary.json", encoding="utf-8"))
        events.append(dict(t=summary["race_start_sim_s"], what="RGB 출발 기록(live_summary race_start_sim_s)"))
        for level in (10, 20, 30, 40):
            hit = next((r["t"] for r in traj if (r.get("progress_m") or 0) >= level), None)
            if hit is not None:
                events.append(dict(t=hit, what=f"진행 {level} m 통과(trajectory progress_m)"))
        ego = np.array(load_jsonl(run / "ego_poses.jsonl"), dtype=float)
        grid = np.arange(seg["cut"][0], seg["cut"][1], 0.01)
        px, py = np.interp(grid, ego[:, 0], ego[:, 1]), np.interp(grid, ego[:, 0], ego[:, 2])
        speed = np.hypot(np.gradient(px, grid), np.gradient(py, grid))
        speed = np.convolve(speed, np.ones(5) / 5, mode="same")
        events.append(dict(t=float(grid[int(np.argmax(speed))]),
                           what=f"자세 차분 속도 최대 부근(약 {speed.max():.2f} m/s; 원 보고 최고 2.862 m/s)"))
        low = summary["low_samples_after_half_meter"]
        events.append(dict(t=low[0][0], what=f"후반 짧은 저속 표본 시작({low[0][1]:.2f} m 부근)"))
        events.append(dict(t=summary["race_start_sim_s"] + summary["elapsed_to_goal_sample_s"],
                           what="목표 도달 표본(영상 마지막 프레임 37.50 s 뒤 0.023 s)"))
        return sorted(events, key=lambda e: e["t"])
    green = next((r["t"] for r in traj if r.get("light") == "green"), None)
    if green is not None:
        events.append(dict(t=green, what="녹색 신호 관측(trajectory light=green 첫 표본)"))
    last = None
    for r in traj:
        behavior = (r.get("autonomy") or {}).get("behavior")
        if behavior != last and behavior in ("MOVING_PASS", "BYPASS", "FOLLOW") and last is not None:
            events.append(dict(t=r["t"], what=f"제어 상태 {last}→{behavior}"))
        last = behavior if behavior not in ("HOLD", None) else last
    if seg["actor"]:
        ego = np.array(load_jsonl(run / "ego_poses.jsonl"), dtype=float)
        actor = np.array(load_jsonl(run / "actor_poses.jsonl"), dtype=float)
        t = np.arange(seg["cut"][0], seg["cut"][1], 0.01)
        ex, ey = np.interp(t, ego[:, 0], ego[:, 1]), np.interp(t, ego[:, 0], ego[:, 2])
        eyaw = np.interp(t, ego[:, 0], np.unwrap(ego[:, 3]))
        ax, ay = np.interp(t, actor[:, 0], actor[:, 1]), np.interp(t, actor[:, 0], actor[:, 2])
        lon = np.cos(eyaw) * (ax - ex) + np.sin(eyaw) * (ay - ey)
        lat = -np.sin(eyaw) * (ax - ex) + np.cos(eyaw) * (ay - ey)
        # 차체 길이 0.2 m: 상대 중심이 자차 중심보다 +0.2 m 앞이면 상대 뒤끝=자차 앞끝.
        labels = {(0.2, -1): "종방향 겹침 시작: 자차 앞끝이 상대 뒤끝에 도달",
                  (0.2, 1): "종방향 겹침 끝: 상대가 다시 자차 앞으로 벗어남",
                  (0.0, -1): "두 차 중심 나란히(자차가 앞으로)",
                  (0.0, 1): "두 차 중심 나란히(상대가 앞으로)",
                  (-0.2, -1): "종방향 겹침 끝: 자차 뒤끝이 상대 앞끝을 지남(자차 완전히 앞섬)",
                  (-0.2, 1): "종방향 겹침 시작: 뒤 상대 앞끝이 자차 뒤끝에 도달"}
        for level in (0.2, 0.0, -0.2):
            sign = np.sign(lon - level)
            last_t = -1e9
            for i in np.nonzero(np.diff(sign) != 0)[0]:
                # 100 Hz 계단 자세로 생기는 0.2 s 이내 반복 교차는 첫 교차 하나로 본다.
                if abs(lat[i]) < 1.0 and t[i] - last_t > 0.2:
                    direction = 1 if lon[min(i + 30, len(lon) - 1)] > lon[max(i - 30, 0)] else -1
                    events.append(dict(t=float(t[i]), what=f"{labels[(level, direction)]}, 중심 횡간격 {abs(lat[i]):.2f} m",
                                       rel_lon_m=round(float(lon[i]), 3)))
                    last_t = t[i]
        # 상대 속도 변화(1초 창 이동량/1초, 50 Hz 순간이동·100 Hz 계단 기록이므로 순간 미분을 쓰지 않음).
        speed = np.hypot(np.interp(t + 0.5, actor[:, 0], actor[:, 1]) - np.interp(t - 0.5, actor[:, 0], actor[:, 1]),
                         np.interp(t + 0.5, actor[:, 0], actor[:, 2]) - np.interp(t - 0.5, actor[:, 0], actor[:, 2]))
        for level, up, down in ((0.05, "상대 재출발(1초 이동속도 0.05 m/s 초과)", "상대 정지(1초 이동속도 0.05 m/s 미만)"),
                                (0.40, "상대 가속(1초 이동속도 0.40 m/s 초과)", "상대 감속(1초 이동속도 0.40 m/s 미만)")):
            sign = np.sign(speed - level)
            for i in np.nonzero(np.diff(sign) != 0)[0]:
                events.append(dict(t=float(t[i]), what=up if sign[i + 1] > 0 else down,
                                   actor_speed_1s_mps=round(float(speed[i + 1]), 3)))
    events = [e for e in events if seg["cut"][0] - 1e-6 <= e["t"] <= seg["cut"][1] + 1e-6]
    return sorted(events, key=lambda e: e["t"])


def write_edit_markdown(path, result):
    lines = ["# 편집표 — feature_showcase.mp4", "",
             f"자동 생성(`tools/compose_showcase.py`). 전체 {result['frames']}프레임·{result['duration_s']:.2f} s·20 fps·1280×720, "
             "모든 장면 1.0배속(프레임 1장=시뮬레이션 0.05 s). 사건 시각은 원시 기록에서 계산했으며 "
             "판정 결과는 각 원 보고서를 따른다. 상대차 자세·정답 자세는 영상/사후 설명에만 쓰였다.", ""]
    for seg in result["segments"]:
        lines += [f"## {seg['title']} — 영상 {seg['video_start_s']:.2f}–{seg['video_end_s']:.2f} s",
                  "", f"- 원자료: `{seg['source_run']}` ({'원본 3인칭 녹화' if seg['kind'] == 'video' else '기록 자세 Gazebo 재렌더'})",
                  f"- 원 시뮬레이션 시각: {seg['sim_first_frame_s']:.2f}–{seg['sim_last_frame_s']:.2f} s, {seg['frames']}프레임",
                  f"- 보고서: `{seg['report']}`", "", "| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |", "|---:|---:|---|"]
        for e in seg["events"]:
            video = f"{e['video_s']:.2f} s" if e["in_clip"] else "구간 밖"
            lines.append(f"| {video} | {e['t']:.3f} s | {e['what']} |")
        lines.append("")
    path.write_text(chr(10).join(lines) + chr(10), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out = args.out.resolve()
    out_mp4 = args.out / "feature_showcase.mp4"
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    cmd = [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-preset", "slow", "-crf", "20",
           "-pix_fmt", "yuv420p", "-profile:v", "high", "-movflags", "+faststart", "-r", str(FPS), str(out_mp4)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    table, index = [], 0
    for seg in SEGMENTS:
        frames, source = video_frames(seg) if seg["kind"] == "video" else render_frames(seg, args.work)
        times = np.array([t for t, _ in frames])
        gaps = np.diff(times)
        if len(frames) < 2 or np.max(np.abs(gaps - DT)) > 1e-3:
            raise ValueError(f"{seg['key']}: non-uniform source time step {gaps.min() if len(gaps) else None}-{gaps.max() if len(gaps) else None}")
        start_index = index
        for t, frame in frames:
            proc.stdin.write(overlay(frame, seg, t).tobytes())
            index += 1
        out_start = start_index * DT
        events = scene_events(seg)
        for e in events:
            e["in_clip"] = bool(times[0] - 1e-6 <= e["t"] <= times[-1] + 1e-6)
            e["video_s"] = round(out_start + (e["t"] - times[0]), 3) if e["in_clip"] else None
            e["t"] = round(e["t"], 3)
        table.append(dict(key=seg["key"], title=seg["title"], run=seg["run"], kind=seg["kind"],
                          source_run=f"artifacts/validation/{seg['run_dir']}",
                          report=f"artifacts/validation/{seg['report']}",
                          frames=len(frames), video_start_s=round(out_start, 3),
                          video_end_s=round(index * DT, 3), sim_first_frame_s=round(float(times[0]), 3),
                          sim_last_frame_s=round(float(times[-1]), 3), playback_rate=1.0,
                          source=source, events=events))
        print(seg["key"], len(frames), f"{out_start:.2f}-{index * DT:.2f}s")
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError("ffmpeg failed")
    digest = hashlib.sha256(out_mp4.read_bytes()).hexdigest()
    result = dict(output=str(out_mp4.relative_to(REPO)).replace("\\", "/"), sha256=digest, frames=index,
                  duration_s=round(index * DT, 3), fps=FPS, size=[W, H],
                  encoder=dict(codec="libx264 (imageio-ffmpeg 9.0.2 static build)", crf=20, preset="slow",
                               pix_fmt="yuv420p", movflags="+faststart", audio=None),
                  segments=table)
    (args.out / "edit_table.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    write_edit_markdown(args.out / "edit_table.md", result)
    print(json.dumps(dict(frames=index, duration_s=round(index * DT, 3), sha256=digest)))


if __name__ == "__main__":
    main()
