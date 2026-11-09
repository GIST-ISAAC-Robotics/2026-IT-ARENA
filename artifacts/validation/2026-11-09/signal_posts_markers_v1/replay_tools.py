"""저장 판단 입력 재생 공용 도구(평가 전용). 실제 경로 변화/폐루프 검증이 아니다."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
R25 = ROOT / "artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1"
WORLD = ROOT / "src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf"
LIDAR_XY = (0.060, 0.0)   # b0390_c1_jr60p20_sim.yaml lidar xyz_m, 모델 원점 기준


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_launch():
    spec = importlib.util.spec_from_file_location("signal_posts_launch_eval", ROOT / "src/arena_bringup/launch/simulation.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_controller(header):
    from arena_autonomy.local_follow import FollowConfig, LocalFollow
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
    cfg = FollowConfig(**header["follow_config"])
    mc = MotionConfig(**header["motion_config"])
    controller = (BypassHandoffFollow(cfg, HandoffExecutionConfig.from_mapping(header["execution_config"]),
                                      allow_simulation_trial=True)
                  if "execution_config" in header else LocalFollow(cfg))
    return controller, mc


def read_events(path):
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            yield json.loads(line)


class Poses:
    """평가기 정답 자세(약 2 ms). 합성 센서 반환 생성에만 쓰며 제어 입력이 아니다."""

    def __init__(self, path=R25 / "ego_poses.jsonl"):
        rows = np.array([json.loads(line) for line in open(path, encoding="utf-8")], dtype=float)
        self.t, self.x, self.y = rows[:, 0], rows[:, 1], rows[:, 2]
        self.yaw = np.unwrap(rows[:, 3])

    def at(self, times):
        return (np.interp(times, self.t, self.x), np.interp(times, self.t, self.y),
                np.interp(times, self.t, self.yaw))


def scan_geometry(scan, poses):
    """광선별 세계 원점·방향. 시각은 stamp + i*time_increment(lidar_motion과 같음)."""
    n = len(scan["ranges"])
    stamp = scan["sec"] + scan["nanosec"] * 1e-9
    times = stamp + np.arange(n) * scan["time_increment"]
    x, y, yaw = poses.at(times)
    ox = x + LIDAR_XY[0] * np.cos(yaw) - LIDAR_XY[1] * np.sin(yaw)
    oy = y + LIDAR_XY[0] * np.sin(yaw) + LIDAR_XY[1] * np.cos(yaw)
    angle = yaw + scan["angle_min"] + np.arange(n) * scan["angle_increment"]
    return ox, oy, np.cos(angle), np.sin(angle)


def ray_box(ox, oy, dx, dy, cx, cy, yaw, along, across):
    """회전 상자(로컬 x=along, y=across)와 광선의 첫 교차 거리. 없으면 inf."""
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = (ox - cx) * c + (oy - cy) * s, -(ox - cx) * s + (oy - cy) * c
    vx, vy = dx * c + dy * s, -dx * s + dy * c
    lo, hi = np.full(len(ox), -np.inf), np.full(len(ox), np.inf)
    for p, v, half in ((px, vx, along / 2), (py, vy, across / 2)):
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (-half - p) / v, (half - p) / v
        parallel = np.abs(v) < 1e-12
        t1 = np.where(parallel, np.where(np.abs(p) <= half, -np.inf, np.inf), t1)
        t2 = np.where(parallel, np.where(np.abs(p) <= half, np.inf, -np.inf), t2)
        lo, hi = np.maximum(lo, np.minimum(t1, t2)), np.minimum(hi, np.maximum(t1, t2))
    hit = (hi >= lo) & (hi > 0)
    return np.where(hit, np.where(lo > 0, lo, np.inf), np.inf)


def finite_ranges(values):
    return np.array([v if isinstance(v, (int, float)) else np.nan for v in values], dtype=float)


def output_row(seq, output, recorded=None):
    det = output[2] if isinstance(output[2], dict) else {}
    row = dict(seq=seq, speed=float(output[0]), steer=float(output[1]), reason=det.get("reason"),
               behavior=det.get("behavior"))
    if recorded is not None:
        row.update(recorded_speed=float(recorded[0]), recorded_steer=float(recorded[1]))
    return row
