#!/usr/bin/env python3
"""보존된 공식 릴리스에서 GitHub 이슈용 기하 증거 그림을 재생성합니다."""

import csv
import io
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, Polygon


ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / "assets/track/official/v2026.08.31/it_arena_track_v2026.08.31.zip"
OUTPUT = ROOT / "artifacts/screenshots/2026-08-31/official_update"

font_path = Path("/mnt/c/Windows/Fonts/malgun.ttf")
if font_path.is_file():
    font_manager.fontManager.addfont(str(font_path))
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font_path)).get_name()
plt.rcParams["axes.unicode_minus"] = False


def rectangle(cx, cy, length, width, yaw):
    points = np.array([
        [-length / 2, -width / 2],
        [length / 2, -width / 2],
        [length / 2, width / 2],
        [-length / 2, width / 2],
    ])
    rotation = np.array([
        [math.cos(yaw), -math.sin(yaw)],
        [math.sin(yaw), math.cos(yaw)],
    ])
    return Polygon(points @ rotation.T + np.array([cx, cy]))


def add_polygon(axis, polygon, **kwargs):
    axis.add_patch(MplPolygon(np.asarray(polygon.exterior.coords), closed=True, **kwargs))


with ZipFile(ARCHIVE) as source:
    scene = json.loads(source.read("output_final/scene.json"))
    branch_rows = list(csv.DictReader(io.StringIO(source.read("output_final/branch_1.csv").decode("utf-8"))))
    world = ET.fromstring(source.read("output_final/world.sdf"))
OUTPUT.mkdir(parents=True, exist_ok=True)

# Evidence 1: official ID 30 board intersects the 20 x 15 cm vehicle envelope at branch 2.
branch = scene["branches"][1]
center = np.asarray([[float(row["x_m"]), float(row["y_m"])] for row in branch_rows], dtype=float)
corridor = LineString(center).buffer(branch["width_m"] / 2, cap_style=2, join_style=2)
marker_data = next(item for item in scene["aruco_markers"]["markers"] if item["id"] == 30)
pose = marker_data["pose"]
marker_link = world.find("./world/model/link[@name='aruco_30']")
marker_pose = [float(value) for value in marker_link.findtext("pose").split()]
marker_size = [float(value) for value in marker_link.findtext("collision/geometry/box/size").split()]
marker = rectangle(marker_pose[0], marker_pose[1], marker_size[0], marker_size[1], marker_pose[5])
blocked = []
for index, point in enumerate(center):
    before = center[max(0, index - 1)]
    after = center[min(len(center) - 1, index + 1)]
    yaw = math.atan2(after[1] - before[1], after[0] - before[0])
    vehicle = rectangle(point[0], point[1], .20, .15, yaw)
    if vehicle.intersects(marker):
        blocked.append((index, vehicle))

fig, ax = plt.subplots(figsize=(10, 5.4), constrained_layout=True)
add_polygon(ax, corridor, facecolor="#4f5660", edgecolor="#1d242b", linewidth=2, label="공식 지름길 2: 폭 0.20 m")
ax.plot(center[:, 0], center[:, 1], "--", color="#f5ca45", linewidth=1.5, label="지름길 중심선")
for _, vehicle in blocked:
    add_polygon(ax, vehicle, facecolor="#e64949", edgecolor="#7f1515", alpha=.18, linewidth=.8)
add_polygon(ax, marker, facecolor="#0c0c0c", edgecolor="#ff78bd", linewidth=3, label="공식 ID 30 판 충돌체: 폭 0.10 m")
ax.scatter([pose["x"]], [pose["y"]], color="#ff78bd", s=50, zorder=5)
ax.annotate(
    f"20×15 cm 차량 외형과 겹치는\n중심선 표본 {len(blocked)}개",
    xy=(pose["x"], pose["y"]), xytext=(2.85, 2.62),
    arrowprops={"arrowstyle": "->", "color": "#ff78bd", "linewidth": 2},
    color="#8a0e57", fontsize=11, weight="bold",
)
ax.set_title("공식 v2026.08.31: ArUco ID 30과 지름길 2 진입 차량의 정적 간섭")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_aspect("equal")
ax.set_xlim(2.05, 3.35)
ax.set_ylim(2.18, 2.72)
ax.grid(alpha=.25)
ax.legend(loc="lower right")
fig.savefig(OUTPUT / "upstream_id30_collision_plan.png", dpi=180)
plt.close(fig)

# Evidence 2: raw SDF swaps the longitudinal and lateral bump dimensions.
bump = scene["speed_bumps"]["bumps"][0]
cx, cy, yaw = bump["x"], bump["y"], bump["yaw_rad"]
bump_link = world.find("./world/model/link[@name='bump_0']")
bump_size = [float(value) for value in bump_link.findtext("collision/geometry/box/size").split()]
raw = rectangle(cx, cy, bump_size[0], bump_size[1], yaw)
intended = rectangle(cx, cy, scene["speed_bumps"]["bump_length_m"], scene["track"]["width_m"], yaw)
track_center = np.asarray(scene["track"]["centerline_polyline"], dtype=float)
distances = np.linalg.norm(track_center - np.array([cx, cy]), axis=1)
near = track_center[distances < .8]
road = LineString(near).buffer(scene["track"]["width_m"] / 2, cap_style=2, join_style=2)

fig, ax = plt.subplots(figsize=(8.5, 6), constrained_layout=True)
add_polygon(ax, road, facecolor="#4f5660", edgecolor="#1d242b", linewidth=2, label="공식 본선: 폭 0.45 m")
ax.plot(near[:, 0], near[:, 1], "--", color="#f5ca45", linewidth=1.5, label="주행 중심선")
add_polygon(ax, raw, facecolor="#e64949", edgecolor="#8b1010", alpha=.68, linewidth=2.2, label="원본 SDF: 진행 45 cm × 횡단 5 cm")
add_polygon(ax, intended, facecolor="none", edgecolor="#3fc477", linewidth=3, linestyle="--", label="설명 기준: 진행 5 cm × 횡단 45 cm")
ax.annotate("진행 방향으로 길게 놓인\n원본 방지턱 충돌 상자", xy=(cx, cy + .21), xytext=(cx + .25, cy + .27), arrowprops={"arrowstyle": "->", "color": "#8b1010"}, color="#8b1010", weight="bold", fontsize=10)
ax.set_title("공식 v2026.08.31: world.sdf 방지턱 치수 방향 비교")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_aspect("equal")
ax.set_xlim(cx - .65, cx + .65)
ax.set_ylim(cy - .65, cy + .65)
ax.grid(alpha=.25)
ax.legend(loc="upper left")
fig.savefig(OUTPUT / "upstream_bump_axis_plan.png", dpi=180)
plt.close(fig)

print(f"blocked_vehicle_samples={len(blocked)}")
