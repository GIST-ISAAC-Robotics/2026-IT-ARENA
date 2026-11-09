"""기존 교통 실행의 Gazebo 기록 자세로 영상 재렌더용 월드와 자세 파일을 만든다.

- 입력(읽기 전용): run 폴더의 runtime_world.sdf, runtime_vehicle.sdf, evaluation_actor.sdf,
  batched_*.obj, ego_poses.jsonl, actor_poses.jsonl.
- 자차·상대 자세는 기록값을 선형 보간만 한다. 각본 목표나 계획 경로를 쓰지 않는다.
- 관찰 카메라 경로만 이 스크립트가 계산한다(평활화한 추적 시점, 주행 판단과 무관).
- 물리/제어/센서 시스템은 제거하고 렌더 센서만 남긴다.

사용: python3 build_replay_scene.py --run RUN --t0 T0 --t1 T1 --out OUT --plugin-dir DIR
"""
import argparse
import copy
import json
import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

EGO_Z = 0.003      # 노면 OBJ 최고 높이(모델 원점이 바퀴 바닥)
ACTOR_Z = 0.06     # evaluation_actor.sdf 원래 pose z
LEAD_S = 1.0       # 렌더 준비용 선행 시간(최종 영상에는 사용하지 않음)
TAIL_S = 0.2
CAM_FOV = 1.20
CAM_W, CAM_H = 1280, 720
CAM_RATE = 20.0


def load(path):
    rows = [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    return np.array(rows, dtype=float)


def interp_pose(arr, t):
    yaw = np.unwrap(arr[:, 3])
    return (np.interp(t, arr[:, 0], arr[:, 1]), np.interp(t, arr[:, 0], arr[:, 2]),
            np.interp(t, arr[:, 0], yaw))


def gaussian_smooth(values, dt, sigma_s):
    if sigma_s <= 0:
        return values
    radius = int(math.ceil(3 * sigma_s / dt))
    k = np.exp(-0.5 * (np.arange(-radius, radius + 1) * dt / sigma_s) ** 2)
    k /= k.sum()
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.convolve(padded, k, mode="valid")


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    roll = math.atan2(m[2, 1], m[2, 2])
    yaw = math.atan2(m[1, 0], m[0, 0])
    return roll, pitch, yaw


def parse_pose(element):
    if element is None or not (element.text or "").strip():
        return np.zeros(6)
    return np.array([float(v) for v in element.text.split()])


def compose(a, b):
    ra, rb = rpy_matrix(*a[3:]), rpy_matrix(*b[3:])
    pos = a[:3] + ra @ b[:3]
    return np.concatenate([pos, matrix_rpy(ra @ rb)])


def fmt(values):
    return " ".join(f"{v:.6f}" for v in values)


def vehicle_visual_model(vehicle_sdf):
    root = ET.parse(vehicle_sdf).getroot()
    src = root.find("model")
    model = ET.Element("model", name="replay_ego")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="body")
    count = 0
    for src_link in src.findall("link"):
        link_pose = parse_pose(src_link.find("pose"))
        for visual in src_link.findall("visual"):
            out = copy.deepcopy(visual)
            out.set("name", f"{src_link.get('name')}__{visual.get('name')}")
            for pose in out.findall("pose"):
                out.remove(pose)
            pose = ET.SubElement(out, "pose")
            pose.text = fmt(compose(link_pose, parse_pose(visual.find("pose"))))
            link.append(out)
            count += 1
    return model, count


def actor_visual_model(actor_sdf):
    root = ET.parse(actor_sdf).getroot()
    src = root.find("model")
    model = ET.Element("model", name="replay_actor")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="body")
    visuals = src.find("link").findall("visual")
    for visual in visuals:
        link.append(copy.deepcopy(visual))
    return model, len(visuals)


def apply_signal_state(world, sequence_path, t0, t1):
    """기록된 실제 적용 신호 단계를 [t0, t1] 동안 고정 표시한다(구간 중 변경이 있으면 거부, 선행 렌더 구간은 영상에 쓰지 않음)."""
    applied = []
    for line in open(sequence_path, encoding="utf-8"):
        row = json.loads(line)
        if "stage" in row:
            applied.append((row["applied_sim_s"], row["stage"], {r["lamp"] for r in row["receipts"] if r["lit"]}))
    before = [a for a in applied if a[0] <= t0]
    if any(t0 < a[0] <= t1 for a in applied):
        raise ValueError("signal stage changes inside clip window")
    stage, lit = (before[-1][1], before[-1][2]) if before else ("initial", set())
    for visual in world.iter("visual"):
        name = visual.get("name", "")
        if not name.startswith("lamp_") or not name.endswith("_vis"):
            continue
        lamp = name[:-4]
        on = lamp in lit
        color = ("0 1 0" if "green" in lamp else "1 0 0") if on else ".004 .004 .004"
        material = visual.find("material")
        for key, value in (("ambient", color + " 1"), ("diffuse", color + " 1"),
                           ("emissive", (color if on else "0 0 0") + " 1")):
            element = material.find(key)
            if element is None:
                element = ET.SubElement(material, key)
            element.text = value
    return dict(stage=stage, applied_sim_s=before[-1][0] if before else None, lit=sorted(lit))


def camera_model():
    model = ET.fromstring(
        f"""<model name="replay_cam"><static>true</static><pose>0 0 5 0 1.2 0</pose><link name="link">
        <sensor name="showcase_cam" type="camera"><camera><horizontal_fov>{CAM_FOV}</horizontal_fov>
        <image><width>{CAM_W}</width><height>{CAM_H}</height><format>R8G8B8</format></image>
        <clip><near>0.03</near><far>40</far></clip></camera><always_on>true</always_on>
        <update_rate>{CAM_RATE}</update_rate><visualize>false</visualize><topic>/showcase/cam</topic></sensor>
        </link></model>""")
    return model


CAM_PITCH = 0.73          # 약 42° 내려다보는 추적 시점(신호 갠트리 높이 0.4 m보다 충분히 위)
CAM_RANGE = (1.35, 4.0)   # 초점까지 거리 범위 [m]
FRAME_MARGIN = 0.78       # 관심점이 화면 반각의 78% 안에 들어오게 함(차체 크기 여유)
ACTOR_INCLUDE_M = 2.6     # 이 거리 안의 상대차는 화면에 함께 담는다
LOOK_AHEAD_M = 0.7        # 자차 앞쪽도 항상 화면에 둔다


def camera_track(tau, ego, actor):
    """관찰 카메라: 평활한 진행 방향 뒤·위에서, 자차와 가까운 상대차(없으면 자차 앞쪽)가 들어오는 최소 거리로 따라간다."""
    dt = tau[1] - tau[0]
    ex, ey, eyaw = ego
    ax, ay = actor
    yaw_s = gaussian_smooth(eyaw, dt, 0.6)
    fx, fy = np.cos(yaw_s), np.sin(yaw_s)
    near = np.hypot(ax - ex, ay - ey) < ACTOR_INCLUDE_M
    # 상대차가 가까우면 두 차만, 아니면 자차와 자차 앞쪽을 담는다.
    ahead = np.where(near, 0.0, LOOK_AHEAD_M)
    pts_x = np.stack([ex, ex + ahead * fx, np.where(near, ax, ex)], axis=1)
    pts_y = np.stack([ey, ey + ahead * fy, np.where(near, ay, ey)], axis=1)
    focus_x = 0.5 * (pts_x.min(axis=1) + pts_x.max(axis=1))
    focus_y = 0.5 * (pts_y.min(axis=1) + pts_y.max(axis=1))
    focus_x, focus_y = (gaussian_smooth(v, dt, 0.35) for v in (focus_x, focus_y))
    cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    tan_h = math.tan(CAM_FOV / 2) * FRAME_MARGIN
    tan_v = math.tan(CAM_FOV / 2) * CAM_H / CAM_W * FRAME_MARGIN
    ranges = np.arange(CAM_RANGE[0], CAM_RANGE[1] + 1e-9, 0.02)
    best = np.full(len(tau), CAM_RANGE[1])
    for i in range(len(tau)):
        d = np.array([cp * fx[i], cp * fy[i], -sp])
        left = np.array([-fy[i], fx[i], 0.0])
        up = np.cross(d, left)
        focus = np.array([focus_x[i], focus_y[i], 0.05])
        rel = np.stack([pts_x[i] - focus[0], pts_y[i] - focus[1], np.full(3, 0.06 - focus[2])], axis=1)
        for r in ranges:
            q = rel + r * d  # 카메라 원점 기준 관심점
            depth = q @ d
            if np.all(depth > 0.1) and np.all(np.abs(q @ left) <= tan_h * depth) and np.all(np.abs(q @ up) <= tan_v * depth):
                best[i] = r
                break
    # 평활 때문에 필요한 거리보다 가까워지지 않도록 ±0.6 s 최대값을 먼저 취한다.
    k = int(round(0.6 / dt))
    padded = np.pad(best, (k, k), mode="edge")
    held = np.array([padded[i:i + 2 * k + 1].max() for i in range(len(best))])
    r_s = gaussian_smooth(held, dt, 0.5)
    cx = focus_x - r_s * cp * fx
    cy = focus_y - r_s * cp * fy
    cz = 0.05 + r_s * sp
    yaw = np.arctan2(fy, fx)
    return np.stack([cx, cy, cz, np.zeros_like(cx), np.full_like(cx, CAM_PITCH), yaw], axis=1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--t0", type=float, required=True)
    parser.add_argument("--t1", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--plugin-dir", type=Path, required=True)
    args = parser.parse_args()
    run, out = args.run, args.out
    out.mkdir(parents=True, exist_ok=True)
    (out / "frames").mkdir(exist_ok=True)

    ego_arr, actor_arr = load(run / "ego_poses.jsonl"), load(run / "actor_poses.jsonl")
    start = args.t0 - LEAD_S
    duration = args.t1 - start + TAIL_S
    tau = np.arange(0.0, duration + 1e-9, 0.01)
    t = start + tau
    if t[0] < ego_arr[0, 0] or t[-1] > ego_arr[-1, 0] or t[-1] > actor_arr[-1, 0]:
        raise ValueError("clip window outside recorded poses")
    ego = interp_pose(ego_arr, t)
    actor = interp_pose(actor_arr, t)
    cam = camera_track(tau, ego, actor[:2])

    lines = ["# name tau x y z roll pitch yaw  (tau = render sim time; original sim time = tau + %.6f)" % start]
    # 기록 표본 시각을 그대로 렌더 시각으로 옮겨 보간 왜곡을 줄인다.
    for name, arr, z in (("replay_ego", ego_arr, EGO_Z), ("replay_actor", actor_arr, ACTOR_Z)):
        mask = (arr[:, 0] >= start - 0.05) & (arr[:, 0] <= start + duration + 0.05)
        for row in arr[mask]:
            lines.append(f"{name} {row[0] - start:.6f} {row[1]:.6f} {row[2]:.6f} {z:.4f} 0 0 {row[3]:.6f}")
    for i in range(len(tau)):
        lines.append("replay_cam %.6f %s" % (tau[i], fmt(cam[i])))
    (out / "poses.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    world_text = (run / "runtime_world.sdf").read_text(encoding="utf-8")
    for obj in sorted(run.glob("batched_*.obj")):
        shutil.copy2(obj, out / obj.name)
    world_text = re.sub(r"<uri>/tmp/arena_world_[^/]+/(batched_[^<]+\.obj)</uri>",
                        lambda m: f"<uri>{(out / m.group(1)).as_posix()}</uri>", world_text)
    root = ET.fromstring(world_text)
    world = root.find("world")
    keep = {"gz-sim-sensors-system"}
    for plugin in list(world.findall("plugin")):
        if plugin.get("filename") not in keep:
            world.remove(plugin)
    for model in list(world.findall("model")):
        if model.get("name") == "eval_g1b_actor":
            world.remove(model)
    lamp_state = apply_signal_state(world, run / "signal_sequence.jsonl", args.t0, args.t1)
    ego_model, ego_visuals = vehicle_visual_model(run / "runtime_vehicle.sdf")
    actor_model, actor_visuals = actor_visual_model(run / "evaluation_actor.sdf")
    world.append(ego_model)
    world.append(actor_model)
    world.append(camera_model())
    plugin = ET.SubElement(world, "plugin", filename=str(args.plugin_dir / "libshowcase-pose-replay.so"),
                           name="showcase::PoseReplay")
    for key, value in (("pose_file", (out / "poses.txt").as_posix()), ("frame_dir", (out / "frames").as_posix()),
                       ("done_file", (out / "done.txt").as_posix()), ("end_time", f"{duration:.3f}"),
                       ("image_topic", "/showcase/cam")):
        ET.SubElement(plugin, key).text = value
    ET.ElementTree(root).write(out / "render_world.sdf", encoding="utf-8", xml_declaration=True)
    meta = dict(run=str(run), t0=args.t0, t1=args.t1, render_tau_to_sim_offset_s=start, render_duration_s=duration,
                ego_visuals=ego_visuals, actor_visuals=actor_visuals, signal=lamp_state, ego_z=EGO_Z, actor_z=ACTOR_Z,
                camera=dict(hfov_rad=CAM_FOV, width=CAM_W, height=CAM_H, rate_hz=CAM_RATE),
                ego_samples=int(((ego_arr[:, 0] >= args.t0) & (ego_arr[:, 0] <= args.t1)).sum()),
                actor_samples=int(((actor_arr[:, 0] >= args.t0) & (actor_arr[:, 0] <= args.t1)).sum()))
    (out / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta))


if __name__ == "__main__":
    main()
