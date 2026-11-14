"""기존 Gazebo 실행의 저장 자세로 기록 영상 재렌더 장면(월드·자세·카메라 경로)을 만든다.

2026-11-09 feature_showcase/tools/build_replay_scene.py를 바탕으로 다음을 바꿨다.
- 관찰 카메라: 낮은 경로 추종 chase. 자차가 이미 지나온 기록 경로 위(뒤쪽)에 카메라를 두고,
  자차와 앞쪽 기록 경로(look-ahead)를 함께 본다. 벽(0.4 m) 위에서 다음 코너가 보이도록 높이를 둔다.
  상대차가 가까우면 두 차가 모두 들어오도록 거리만 넓힌다. 카메라 경로는 촬영 전용이다.
- 신호등: 기록된 실제 적용 단계(signal_sequence.jsonl)를 시각대로 바꿔 표시한다(점등 구 모형을 순간 이동).
- 카메라 여러 대(주 화면 + 선택형 신호등 근접 보조 화면), 1920×1080/30 Hz 기본.

입력(읽기 전용): RUN의 runtime_world.sdf, runtime_vehicle.sdf, batched_*.obj, ego_poses.jsonl,
actor_poses.jsonl(+evaluation_actor.sdf 또는 월드 안 eval_g1b_actor), signal_sequence.jsonl.
자차·상대 자세는 기록값의 선형 보간만 한다. 각본 목표·계획 경로·제어기 출력은 쓰지 않는다.
물리/제어/접촉 시스템은 제거하고 렌더 센서만 남긴다.

사용: python3 build_scene.py --run RUN --t0 T0 --t1 T1 --out OUT --plugin PLUGIN_SO [--signal-inset]
"""
import argparse
import copy
import hashlib
import json
import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

EGO_Z = 0.003       # 노면 OBJ 최고 높이(모델 원점이 바퀴 바닥)
LEAD_S = 0.6        # 렌더 준비용 선행 시간(최종 영상에서 잘라 냄)
TAIL_S = 0.2
PHYS_STEP = 0.001
LAMP_OFF_Z = -5.0   # 꺼진 점등 모형은 지면 아래로 숨긴다

# 주 카메라 기본값(관찰 전용)
CAM = dict(hfov=1.05, back=1.30, height=0.80, look_ahead=1.50, target_weight=0.40, target_z=0.06,
           k_max=1.8, margin_x=0.80, margin_y=0.76, actor_near=2.0, actor_far=3.2, actor_behind_m=0.8,
           sigma_heading=0.45, sigma_cam=0.20, sigma_target=0.30, sigma_k=0.50, hold_k=0.70)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quat_yaw(q):
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_poses(path, name=None):
    """[t, x, y, yaw] 행 또는 Pose_V JSON 행을 [t, x, y, yaw, z] 배열로 읽는다. z가 없으면 NaN."""
    rows = []
    for line in open(path, encoding="utf-8"):
        if not line.strip():
            continue
        item = json.loads(line)
        if isinstance(item, list):
            rows.append([item[0], item[1], item[2], item[3], float("nan")])
            continue
        for pose in item.get("pose", []):
            if name and pose.get("name") != name:
                continue
            stamp = pose.get("header", {}).get("stamp", {})
            t = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9
            p = pose.get("position", {})
            rows.append([t, float(p.get("x", 0.0)), float(p.get("y", 0.0)), quat_yaw(pose.get("orientation", {})), float(p.get("z", 0.0))])
    arr = np.array(rows, dtype=float)
    if len(arr):
        arr = arr[np.argsort(arr[:, 0], kind="stable")]
    return arr


def interp_pose(arr, t):
    yaw = np.unwrap(arr[:, 3])
    return np.interp(t, arr[:, 0], arr[:, 1]), np.interp(t, arr[:, 0], arr[:, 2]), np.interp(t, arr[:, 0], yaw)


def gaussian_smooth(values, dt, sigma_s):
    if sigma_s <= 0:
        return values
    radius = int(math.ceil(3 * sigma_s / dt))
    k = np.exp(-0.5 * (np.arange(-radius, radius + 1) * dt / sigma_s) ** 2)
    k /= k.sum()
    return np.convolve(np.pad(values, (radius, radius), mode="edge"), k, mode="valid")


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    return math.atan2(m[2, 1], m[2, 2]), pitch, math.atan2(m[1, 0], m[0, 0])


def parse_pose(element):
    if element is None or not (element.text or "").strip():
        return np.zeros(6)
    return np.array([float(v) for v in element.text.split()])


def compose(a, b):
    ra, rb = rpy_matrix(*a[3:]), rpy_matrix(*b[3:])
    return np.concatenate([a[:3] + ra @ b[:3], matrix_rpy(ra @ rb)])


def fmt(values):
    return " ".join(f"{v:.6f}" for v in values)


def flatten_visual_model(src_model, name):
    """원 모델의 시각 요소만 정적 모델 하나로 옮긴다(link pose ∘ visual pose)."""
    model = ET.Element("model", name=name)
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="body")
    count = 0
    for src_link in src_model.findall("link"):
        link_pose = parse_pose(src_link.find("pose"))
        for visual in src_link.findall("visual"):
            out = copy.deepcopy(visual)
            out.set("name", f"{src_link.get('name')}__{visual.get('name')}")
            for pose in out.findall("pose"):
                out.remove(pose)
            ET.SubElement(out, "pose").text = fmt(compose(link_pose, parse_pose(visual.find("pose"))))
            link.append(out)
            count += 1
    return model, count


def lamp_visuals(world):
    """월드의 신호 점등 구(lamp_*_vis)와 월드 좌표 자세를 찾는다."""
    found = {}
    for model in world.findall("model"):
        mpose = parse_pose(model.find("pose"))
        for link in model.findall("link"):
            lpose = compose(mpose, parse_pose(link.find("pose")))
            for visual in link.findall("visual"):
                vname = visual.get("name", "")
                if vname.startswith("lamp_") and vname.endswith("_vis"):
                    found[vname[:-4]] = (visual, compose(lpose, parse_pose(visual.find("pose"))))
    return found


def panel_pose(world):
    for model in world.findall("model"):
        mpose = parse_pose(model.find("pose"))
        for link in model.findall("link"):
            if link.get("name") == "tl_panel":
                return compose(mpose, parse_pose(link.find("pose")))
    return None


def signal_stages(sequence_path):
    stages = []
    for line in open(sequence_path, encoding="utf-8"):
        row = json.loads(line)
        if "stage" in row:
            stages.append((float(row["applied_sim_s"]), row["stage"], sorted(r["lamp"] for r in row["receipts"] if r["lit"])))
    return stages


def lamp_overlays(world, stages, start, duration):
    """원 점등 구는 모두 소등색으로 두고, 켜진 구 모형을 기록 적용 시각에 맞춰 순간 이동한다."""
    lamps = lamp_visuals(world)
    for visual, _ in lamps.values():
        material = visual.find("material")
        if material is None:
            material = ET.SubElement(visual, "material")
        for key, value in (("ambient", ".004 .004 .004 1"), ("diffuse", ".004 .004 .004 1"), ("emissive", "0 0 0 1")):
            element = material.find(key)
            if element is None:
                element = ET.SubElement(material, key)
            element.text = value
    models, lines = [], []
    for lamp, (visual, pose) in sorted(lamps.items()):
        color = "0 1 0" if "green" in lamp else "1 0 0"
        model = ET.fromstring(
            f"""<model name="lit_{lamp}"><static>true</static><pose>0 0 {LAMP_OFF_Z} 0 0 0</pose><link name="link">
            <visual name="v"><cast_shadows>false</cast_shadows><geometry><sphere><radius>0.0082</radius></sphere></geometry>
            <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse><emissive>{color} 1</emissive></material>
            </visual></link></model>""")
        models.append(model)
        on_pose = fmt([pose[0], pose[1], pose[2], 0, 0, 0])
        off_pose = fmt([pose[0], pose[1], LAMP_OFF_Z, 0, 0, 0])

        def lit_at(t):
            before = [s for s in stages if s[0] <= t]
            return bool(before) and lamp in before[-1][2]

        lines.append(f"lit_{lamp} 0.000000 {on_pose if lit_at(start) else off_pose}")
        state = lit_at(start)
        for applied, _, lit in stages:
            tau = applied - start
            if 0.0 < tau <= duration:
                new = lamp in lit
                if new != state:
                    lines.append(f"lit_{lamp} {tau - 0.0005:.6f} {on_pose if state else off_pose}")
                    lines.append(f"lit_{lamp} {tau:.6f} {on_pose if new else off_pose}")
                    state = new
        lines.append(f"lit_{lamp} {duration + 1.0:.6f} {on_pose if state else off_pose}")
    return models, lines, sorted(lamps)


def camera_model(name, topic, hfov, width, height, rate, near=0.03, far=60):
    return ET.fromstring(
        f"""<model name="{name}"><static>true</static><pose>0 0 5 0 1.2 0</pose><link name="link">
        <sensor name="{name}_sensor" type="camera"><camera><horizontal_fov>{hfov}</horizontal_fov>
        <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
        <clip><near>{near}</near><far>{far}</far></clip></camera>
        <always_on>true</always_on><update_rate>{rate}</update_rate><visualize>false</visualize>
        <topic>{topic}</topic></sensor></link></model>""")


class Path2D:
    """자차 기록 경로를 호 길이로 매개화한다(시작 전/종료 후는 기록 yaw 방향 직선 연장)."""

    def __init__(self, ego):
        t = ego[:, 0]
        self.g = np.arange(t[0], t[-1], 0.005)
        px = gaussian_smooth(np.interp(self.g, t, ego[:, 1]), 0.005, 0.08)
        py = gaussian_smooth(np.interp(self.g, t, ego[:, 2]), 0.005, 0.08)
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(px), np.diff(py)))])
        keep = np.concatenate([[True], np.diff(self.s) > 1e-5])
        S, X, Y = self.s[keep], px[keep], py[keep]
        ext = np.linspace(0.05, 8.0, 160)
        yaw0, yaw1 = ego[0, 3], ego[-1, 3]
        self.S = np.concatenate([-ext[::-1], S, S[-1] + ext])
        self.X = np.concatenate([X[0] - ext[::-1] * math.cos(yaw0), X, X[-1] + ext * math.cos(yaw1)])
        self.Y = np.concatenate([Y[0] - ext[::-1] * math.sin(yaw0), Y, Y[-1] + ext * math.sin(yaw1)])

    def s_at(self, t):
        return np.interp(t, self.g, self.s)

    def xy(self, s):
        return np.interp(s, self.S, self.X), np.interp(s, self.S, self.Y)


def project(cam, target, pts, hfov, aspect):
    f = target - cam
    f = f / np.linalg.norm(f)
    left = np.cross(np.array([0.0, 0.0, 1.0]), f)
    left /= np.linalg.norm(left)
    up = np.cross(f, left)
    q = pts - cam
    depth = q @ f
    tx, ty = math.tan(hfov / 2), math.tan(hfov / 2) / aspect
    return depth, -(q @ left) / np.maximum(depth, 1e-6) / tx, (q @ up) / np.maximum(depth, 1e-6) / ty


def chase_camera(times, path, ego, actors, width, height, cfg):
    """평활한 자차 진행 방향의 뒤·위에서 따라간다. 코너에서는 카메라가 바깥쪽으로 돌아 벽(0.4 m) 위에서 본다.
    시선 목표는 자차와 기록 경로의 앞쪽(look-ahead) 사이. 앞·옆의 상대차만 함께 담는다."""
    dt = times[1] - times[0]
    aspect = width / height
    ex, ey, eyaw = ego
    heading = gaussian_smooth(eyaw, dt, cfg["sigma_heading"])
    hx, hy = np.cos(heading), np.sin(heading)
    s_now = path.s_at(times)
    ax_, ay_ = path.xy(s_now + cfg["look_ahead"])
    w = cfg["target_weight"]
    tx, ty = ex + w * (ax_ - ex), ey + w * (ay_ - ey)
    near_w = np.zeros_like(times)
    actor_xy = None
    if actors:
        weights = []
        for ax, ay in actors:
            dist = np.hypot(ax - ex, ay - ey)
            lon = np.cos(eyaw) * (ax - ex) + np.sin(eyaw) * (ay - ey)
            wgt = np.clip((cfg["actor_far"] - dist) / (cfg["actor_far"] - cfg["actor_near"]), 0.0, 1.0)
            wgt *= np.clip((lon + cfg["actor_behind_m"]) / 0.5, 0.0, 1.0)   # 완전히 뒤로 빠진 상대는 담지 않는다
            weights.append(gaussian_smooth(wgt, dt, 0.3))
        weights = np.array(weights)
        pick = np.argmax(weights, axis=0)                                  # 프레임마다 가장 가까운 앞·옆 상대 하나
        near_w = weights[pick, np.arange(len(times))]
        actor_xy = (np.array([actors[j][0][i] for i, j in enumerate(pick)]),
                    np.array([actors[j][1][i] for i, j in enumerate(pick)]))
        mx, my = 0.5 * (ex + actor_xy[0]), 0.5 * (ey + actor_xy[1])
        tx, ty = tx + 0.6 * near_w * (mx - tx), ty + 0.6 * near_w * (my - ty)
    tx, ty = gaussian_smooth(tx, dt, cfg["sigma_target"]), gaussian_smooth(ty, dt, cfg["sigma_target"])
    tz = np.full_like(tx, cfg["target_z"])

    def cam_at(i, k):
        return np.array([ex[i] - k * cfg["back"] * hx[i], ey[i] - k * cfg["back"] * hy[i], k * cfg["height"]])

    ks = np.arange(1.0, cfg["k_max"] + 1e-9, 0.02)
    need = np.full(len(times), cfg["k_max"])
    for i in range(len(times)):
        c, s_ = math.cos(eyaw[i]), math.sin(eyaw[i])
        pts = [[ex[i] + 0.14 * c, ey[i] + 0.14 * s_, 0.12], [ex[i] - 0.12 * c, ey[i] - 0.12 * s_, 0.03]]
        if near_w[i] > 0.3:
            pts += [[actor_xy[0][i], actor_xy[1][i], 0.13], [actor_xy[0][i], actor_xy[1][i], 0.0]]
        pts = np.array(pts)
        target = np.array([tx[i], ty[i], tz[i]])
        for k in ks:
            depth, u, v = project(cam_at(i, k), target, pts, cfg["hfov"], aspect)
            if np.all(depth > 0.15) and np.all(np.abs(u) <= cfg["margin_x"]) and np.all(np.abs(v) <= cfg["margin_y"]):
                need[i] = k
                break
    h = int(round(cfg["hold_k"] / dt))
    padded = np.pad(need, (h, h), mode="edge")
    held = np.array([padded[i:i + 2 * h + 1].max() for i in range(len(need))])
    k_s = gaussian_smooth(held, dt, cfg["sigma_k"])
    cams = np.array([cam_at(i, k_s[i]) for i in range(len(times))])
    for j in range(3):
        cams[:, j] = gaussian_smooth(cams[:, j], dt, cfg["sigma_cam"])
    dx, dy, dz = tx - cams[:, 0], ty - cams[:, 1], tz - cams[:, 2]
    yaw = np.unwrap(np.arctan2(dy, dx))
    pitch = np.arctan2(-dz, np.hypot(dx, dy))
    out = np.stack([cams[:, 0], cams[:, 1], cams[:, 2], np.zeros_like(yaw), pitch, yaw], axis=1)
    # 검사용: 자차 중심의 화면 좌표, 카메라 회전 속도
    u_all, v_all = [], []
    for i in range(len(times)):
        _, u, v = project(cams[i], np.array([tx[i], ty[i], tz[i]]), np.array([[ex[i], ey[i], 0.07]]), cfg["hfov"], aspect)
        u_all.append(float(u[0]))
        v_all.append(float(v[0]))
    yaw_rate = np.degrees(np.abs(np.gradient(yaw, dt)))
    pitch_deg = np.degrees(pitch)
    qa = dict(ego_u_abs_max=round(max(abs(x) for x in u_all), 3), ego_v_min=round(min(v_all), 3),
              ego_v_max=round(max(v_all), 3), ego_inside_frame_frames=int(sum(abs(a) <= 1 and abs(b) <= 1 for a, b in zip(u_all, v_all))),
              frames=len(times), cam_yaw_rate_deg_s_max=round(float(yaw_rate.max()), 1),
              cam_yaw_rate_deg_s_p95=round(float(np.percentile(yaw_rate, 95)), 1),
              cam_height_m=[round(float(cams[:, 2].min()), 3), round(float(cams[:, 2].max()), 3)],
              cam_pitch_deg=[round(float(pitch_deg.min()), 1), round(float(pitch_deg.max()), 1)],
              zoom_k=[round(float(k_s.min()), 3), round(float(k_s.max()), 3)])
    return out, qa


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--t0", type=float, required=True)
    parser.add_argument("--t1", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--signal-inset", action="store_true")
    parser.add_argument("--cam", default="{}", help="JSON로 카메라 기본값 일부를 바꾼다")
    args = parser.parse_args()
    cfg = dict(CAM, **json.loads(args.cam))
    run, out = args.run, args.out
    out.mkdir(parents=True, exist_ok=True)
    (out / "frames_main").mkdir(exist_ok=True)

    ego_arr = load_poses(run / "ego_poses.jsonl")
    actor_path = run / "actor_poses.jsonl"
    actor_arr = load_poses(actor_path, "eval_g1b_actor") if actor_path.exists() and actor_path.stat().st_size else np.zeros((0, 5))
    has_actor = len(actor_arr) > 0
    start = args.t0 - LEAD_S
    duration = args.t1 - start + TAIL_S
    if start < ego_arr[0, 0] or start + duration > ego_arr[-1, 0]:
        raise ValueError(f"clip window outside recorded ego poses {ego_arr[0, 0]}–{ego_arr[-1, 0]}")
    if has_actor and (start < actor_arr[0, 0] - 0.02 or start + duration > actor_arr[-1, 0]):
        raise ValueError("clip window outside recorded actor poses")
    tau = np.arange(0.0, duration + 1e-9, 1.0 / args.fps)
    t = start + tau
    ego = interp_pose(ego_arr, t)
    extra = {}
    extra_path = run / "additional_actors.json"
    if extra_path.exists():
        for name, rows in json.load(open(extra_path, encoding="utf-8")).get("poses", {}).items():
            arr = np.array([[r[0], r[1], r[2], r[3], float("nan")] for r in rows], dtype=float)
            arr = arr[np.argsort(arr[:, 0], kind="stable")]
            if start < arr[0, 0] - 0.02 or start + duration > arr[-1, 0]:
                raise ValueError(f"clip window outside recorded poses of {name}")
            extra[name] = arr
    actors = ([interp_pose(actor_arr, t)[:2]] if has_actor else []) + [interp_pose(a, t)[:2] for a in extra.values()]
    path = Path2D(ego_arr)
    cam, cam_qa = chase_camera(t, path, ego, actors, args.width, args.height, cfg)

    lines = ["# name tau x y z roll pitch yaw  (tau = render sim time; original sim time = tau + %.6f)" % start]
    mask = (ego_arr[:, 0] >= start - 0.05) & (ego_arr[:, 0] <= start + duration + 0.05)
    for row in ego_arr[mask]:
        lines.append(f"replay_ego {row[0] - start:.6f} {row[1]:.6f} {row[2]:.6f} {EGO_Z:.4f} 0 0 {row[3]:.6f}")

    world_text = (run / "runtime_world.sdf").read_text(encoding="utf-8")
    for obj in sorted(run.glob("batched_*.obj")):
        shutil.copy2(obj, out / obj.name)
    world_text = re.sub(r"<uri>/tmp/arena_world_[^/]+/(batched_[^<]+\.obj)</uri>",
                        lambda m: f"<uri>{(out / m.group(1)).as_posix()}</uri>", world_text)
    root = ET.fromstring(world_text)
    world = root.find("world")
    for plugin in list(world.findall("plugin")):
        if plugin.get("filename") != "gz-sim-sensors-system":
            world.remove(plugin)
    physics = world.find("physics")
    physics.find("max_step_size").text = f"{PHYS_STEP}"
    physics.find("real_time_factor").text = "1000"   # 렌더 전용: 렌더 속도까지 빠르게
    actor_src = None
    extra_src = {}
    for model in list(world.findall("model")):
        if model.get("name") == "eval_g1b_actor":
            actor_src = model
            world.remove(model)
        elif model.get("name") in extra:
            extra_src[model.get("name")] = model
            world.remove(model)
    actor_info = None
    if has_actor:
        actor_sdf = run / "evaluation_actor.sdf"
        if actor_sdf.exists():
            actor_src = ET.parse(actor_sdf).getroot().find("model")
            actor_info = dict(model_source="evaluation_actor.sdf", sha256=sha256(actor_sdf))
        else:
            actor_info = dict(model_source="runtime_world.sdf eval_g1b_actor")
        actor_model, actor_visuals = flatten_visual_model(actor_src, "replay_actor")
        actor_z = float(np.nanmedian(actor_arr[:, 4])) if np.isfinite(actor_arr[:, 4]).any() else parse_pose(actor_src.find("pose"))[2]
        world.append(actor_model)
        amask = (actor_arr[:, 0] >= start - 0.05) & (actor_arr[:, 0] <= start + duration + 0.05)
        for row in actor_arr[amask]:
            lines.append(f"replay_actor {row[0] - start:.6f} {row[1]:.6f} {row[2]:.6f} {actor_z:.4f} 0 0 {row[3]:.6f}")
        actor_info.update(visuals=actor_visuals, z=actor_z, samples=int(amask.sum()))
    extra_info = {}
    for n, (name, arr) in enumerate(extra.items()):
        if name not in extra_src:
            raise ValueError(f"model {name} not found in runtime_world.sdf")
        model, visuals = flatten_visual_model(extra_src[name], f"replay_extra_{n}")
        z = parse_pose(extra_src[name].find("pose"))[2]
        world.append(model)
        emask = (arr[:, 0] >= start - 0.05) & (arr[:, 0] <= start + duration + 0.05)
        for row in arr[emask]:
            lines.append(f"replay_extra_{n} {row[0] - start:.6f} {row[1]:.6f} {row[2]:.6f} {z:.4f} 0 0 {row[3]:.6f}")
        extra_info[name] = dict(model_source="runtime_world.sdf " + name, replay_name=f"replay_extra_{n}",
                                visuals=visuals, z=z, samples=int(emask.sum()))
    vehicle_root = ET.parse(run / "runtime_vehicle.sdf").getroot()
    ego_model, ego_visuals = flatten_visual_model(vehicle_root.find("model"), "replay_ego")
    world.append(ego_model)

    stages = signal_stages(run / "signal_sequence.jsonl")
    lamp_models, lamp_lines, lamp_names = lamp_overlays(world, stages, start, duration)
    for model in lamp_models:
        world.append(model)
    lines += lamp_lines

    cameras = [("frames_main", "/rec/main")]
    world.append(camera_model("rec_cam_main", "/rec/main", cfg["hfov"], args.width, args.height, args.fps))
    for i in range(len(tau)):
        lines.append("rec_cam_main %.6f %s" % (tau[i], fmt(cam[i])))
    inset = None
    if args.signal_inset:
        panel = panel_pose(world)
        if panel is None:
            raise ValueError("signal panel tl_panel not found")
        normal = rpy_matrix(*panel[3:]) @ np.array([1.0, 0.0, 0.0])
        pos = panel[:3] + 0.42 * normal
        look = -normal
        inset_pose = [pos[0], pos[1], pos[2], 0.0, math.atan2(-look[2], math.hypot(look[0], look[1])), math.atan2(look[1], look[0])]
        (out / "frames_signal").mkdir(exist_ok=True)
        cameras.append(("frames_signal", "/rec/signal"))
        world.append(camera_model("rec_cam_signal", "/rec/signal", 0.50, 640, 360, args.fps, near=0.02, far=5))
        lines.append("rec_cam_signal 0.000000 " + fmt(inset_pose))
        lines.append("rec_cam_signal %.6f %s" % (duration + 1.0, fmt(inset_pose)))
        inset = dict(pose=[round(v, 4) for v in inset_pose], hfov=0.50, size=[640, 360], panel_pose=[round(v, 4) for v in panel])

    (out / "poses.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    plugin = ET.SubElement(world, "plugin", filename=str(args.plugin), name="recording::PoseReplay")
    for key, value in (("pose_file", (out / "poses.txt").as_posix()), ("done_file", (out / "done.txt").as_posix()),
                       ("end_time", f"{duration:.3f}"),
                       ("cameras", ";".join(f"{(out / d).as_posix()}|{topic}" for d, topic in cameras))):
        ET.SubElement(plugin, key).text = value
    ET.ElementTree(root).write(out / "render_world.sdf", encoding="utf-8", xml_declaration=True)

    inputs = {name: sha256(run / name) for name in ("runtime_world.sdf", "runtime_vehicle.sdf", "ego_poses.jsonl",
                                                     "signal_sequence.jsonl") if (run / name).exists()}
    for obj in sorted(run.glob("batched_*.obj")):
        inputs[obj.name] = sha256(obj)
    if has_actor:
        inputs["actor_poses.jsonl"] = sha256(actor_path)
    if extra:
        inputs["additional_actors.json"] = sha256(extra_path)
    textures = sorted(set(re.findall(r"<albedo_map>([^<]+)</albedo_map>", world_text)))
    meta = dict(run=str(run), t0=args.t0, t1=args.t1, render_tau_to_sim_offset_s=start, render_duration_s=duration,
                fps=args.fps, size=[args.width, args.height], physics_step_s=PHYS_STEP,
                ego_visuals=ego_visuals, ego_z=EGO_Z, ego_samples_in_clip=int(((ego_arr[:, 0] >= args.t0) & (ego_arr[:, 0] <= args.t1)).sum()),
                actor=actor_info, additional_actors=extra_info or None, signal_stages=[dict(applied_sim_s=a, stage=s, lit=l) for a, s, l in stages],
                lamps=lamp_names, camera=dict(cfg, kind="path-following low chase (observation only)"), camera_qa=cam_qa,
                signal_inset=inset, input_sha256=inputs, external_textures=textures,
                cameras={d: topic for d, topic in cameras})
    (out / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(dict(duration=duration, frames_expected=len(tau), camera_qa=cam_qa, actor=bool(has_actor), extra_actors=len(extra),
                          lamps=len(lamp_names), stages=len(stages)), ensure_ascii=False))


if __name__ == "__main__":
    main()
