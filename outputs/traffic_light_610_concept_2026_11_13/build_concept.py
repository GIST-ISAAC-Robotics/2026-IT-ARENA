"""공식 구형 STL의 가로대만 160 mm 연장한 시안과 비교 그림을 만든다."""
from pathlib import Path
import hashlib
import json
import struct

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT / "artifacts/validation/2026-11-09/signal_post_source_v1/download/STadium_starting_light.stl"
NEW = ROOT / "artifacts/validation/2026-11-12/signal_post_revision_r291_v1/STardium_starting_light_new.stl"
OLD_HASH = "955addee8f1640902f34cf7a49924335aa4d641219d1cdcfa1537fac426c7d57"
NEW_HASH = "04f43b4499cd32b2bbb27b74eb14735df6ff9c069676157a58f33543d7b1856f"
DTYPE = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def load(raw):
    n = struct.unpack("<I", raw[80:84])[0]
    assert len(raw) == 84 + 50 * n
    return np.frombuffer(raw[84:], dtype=DTYPE).copy()


def size(t):
    return np.ptp(t.reshape(-1, 3), axis=0)


def topology(t):
    v, inverse = np.unique(t.reshape(-1, 3), axis=0, return_inverse=True)
    f = inverse.reshape(-1, 3)
    edges = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
    return dict(vertices=len(v), triangles=len(t), edges=len(counts),
                boundary_edges=int(np.sum(counts == 1)), nonmanifold_edges=int(np.sum(counts > 2)))


def cross(t):
    return np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])


def dimension(ax, a, b, y, label, color="#566878"):
    ax.annotate("", (a, y), (b, y), arrowprops=dict(arrowstyle="|-|", color=color, lw=1.2))
    ax.text((a+b)/2, y-13, label, color=color, ha="center", va="top", fontsize=11)


def front(ax, tri, title, subtitle, road_width, outer_width):
    t = tri.astype(float).copy()
    center = (t[:, :, 0].min() + t[:, :, 0].max())/2
    t[:, :, 0] -= center
    t[:, :, 1] -= t[:, :, 1].min()
    normal = cross(t)
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-12)
    base = np.array([.27, .40, .51])
    shade = .65 + .35*np.abs(normal @ np.array([.25, .35, .903]))
    colors = np.clip(base[None, :] * shade[:, None], 0, 1)
    # Z가 큰 면을 마지막에 그려 신호 모듈 전면을 보인다. 삼각형 대각선을 표시하지 않는다.
    order = np.argsort(t[:, :, 2].mean(axis=1))
    poly = PolyCollection(t[order, :, :2], facecolors=colors[order], edgecolors="none", antialiased=False)
    ax.add_collection(poly)
    ax.plot([-430, 430], [-2, -2], color="#ced5dc", lw=1)
    ax.fill_between([-road_width/2, road_width/2], -12, -3, color="#e9edf1")
    dimension(ax, -road_width/2, road_width/2, -27, f"도로 폭 {road_width} mm")
    dimension(ax, -outer_width/2, outer_width/2, -89, f"발판 포함 전체 폭 {outer_width} mm")
    ax.text(0, 555, title, fontsize=17, weight="bold", ha="center", color="#1e3447")
    ax.text(0, 519, subtitle, fontsize=11, ha="center", color="#627181")
    ax.set(xlim=(-435, 435), ylim=(-140, 585), aspect="equal")
    ax.axis("off")


def render(old, proposed):
    plt.rcParams["font.family"] = "Malgun Gothic"
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 2, figsize=(15, 7.3), facecolor="white")
    front(axes[0], old, "공식 구형 · 450 mm 도로 기준", "10월 9일 공유 STL · 원본", 450, 625)
    front(axes[1], proposed, "구형 연장 시안 · 610 mm 도로 기준", "가로대 +160 mm · 양쪽 지주/발판 각각 80 mm 이동", 610, 785)
    fig.text(.5, .035, "높이 465 mm / 중앙 신호 모듈 / 세로 기둥 단면 / 발판 크기는 원본 유지", ha="center", fontsize=12, color="#334b60")
    fig.subplots_adjust(left=.025, right=.975, top=.97, bottom=.10, wspace=.06)
    fig.savefig(HERE / "comparison.png", dpi=160)
    plt.close(fig)

    t = proposed.astype(float).copy()
    t[:, :, 0] -= (t[:, :, 0].min() + t[:, :, 0].max())/2
    t[:, :, 1] -= t[:, :, 1].min()
    # 오른손 좌표: raw X→X, raw Z→-Y, raw Y→Z. 그림에만 적용한다.
    p = np.stack((t[:, :, 0], -t[:, :, 2], t[:, :, 1]), axis=2)
    normal = cross(p)
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-12)
    shade = .55 + .45*np.abs(normal @ np.array([.32, -.65, .69]))
    colors = np.clip(np.array([.38, .51, .61])[None, :] * shade[:, None], 0, 1)
    fig = plt.figure(figsize=(12, 8), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(p, facecolors=colors, edgecolors="none", linewidth=0, antialiased=False))
    ax.set(xlim=(-425, 425), ylim=(-65, 65), zlim=(-12, 490))
    ax.set_box_aspect((850, 130, 502))
    ax.view_init(elev=14, azim=-71)
    ax.set_proj_type("ortho")
    ax.axis("off")
    fig.text(.5, .93, "구형 직각 신호등 · 가로대 160 mm 연장 시안", ha="center", fontsize=18, weight="bold", color="#1e3447")
    fig.text(.5, .055, "전체 폭 785 mm · 높이 465 mm · 앞뒤 57.32 mm  |  형상 확인용 STL", ha="center", fontsize=12, color="#506577")
    fig.subplots_adjust(left=0, right=1, top=.90, bottom=.08)
    fig.savefig(HERE / "concept_isometric.png", dpi=160)
    plt.close(fig)


def main():
    old_bytes, new_bytes = OLD.read_bytes(), NEW.read_bytes()
    assert sha(old_bytes) == OLD_HASH and sha(new_bytes) == NEW_HASH
    source = load(old_bytes)
    old = source["vertices"].astype(float)
    result = source.copy()
    # 중앙 모듈의 실제 범위는 X=-89.220695..70.779305이다.
    # 바깥 양쪽 지주/발판의 모든 꼭짓점을 강체 이동하면 연결된 수평 가로대만 늘어난다.
    move = np.where(old[:, :, 0] < -110, -80., np.where(old[:, :, 0] > 90, 80., 0.))
    result["vertices"][:, :, 0] += move.astype(np.float32)
    proposed = result["vertices"].astype(float)
    changed = np.any(move != 0, axis=1)
    stretched = np.any(move != move[:, :1], axis=1)
    module = np.all((old[:, :, 0] > -110) & (old[:, :, 0] < 90) &
                    (old[:, :, 1] > -60) & (old[:, :, 1] < 16), axis=1)
    # 변경 면만 법선을 재계산하며 중앙 모듈 원 레코드는 바이트 단위로 유지한다.
    normal = cross(proposed[changed])
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    result["normal"][changed] = normal
    assert module.sum() == 7033
    assert result[module].tobytes() == source[module].tobytes()
    assert np.array_equal(old[:, :, 1:], proposed[:, :, 1:])
    assert np.array_equal(proposed[:, :, 0] - old[:, :, 0], move)
    assert np.allclose(size(proposed)-size(old), [160, 0, 0], atol=1e-8)
    assert topology(old) == topology(proposed)
    assert np.all(np.sum(cross(old) * cross(proposed), axis=1) > 0)
    assert np.isclose(old[stretched, :, 1].min()-old[:, :, 1].min(), 450, atol=.00001)
    assert np.isclose(old[stretched, :, 1].max()-old[:, :, 1].min(), 465, atol=.00001)
    rigid = ~stretched
    for a, b in ((0, 1), (1, 2), (2, 0)):
        assert np.allclose(np.linalg.norm(old[rigid, a]-old[rigid, b], axis=1),
                           np.linalg.norm(proposed[rigid, a]-proposed[rigid, b], axis=1), atol=1e-8)
    output = HERE / "signal_610_old_style.stl"
    header = b"IT ARENA concept: old frame +160mm span; raw axes and mm assumed".ljust(80, b" ")
    output.write_bytes(header + struct.pack("<I", len(result)) + result.tobytes())
    assert np.array_equal(load(output.read_bytes())["vertices"], result["vertices"])
    assert sha(OLD.read_bytes()) == OLD_HASH and sha(NEW.read_bytes()) == NEW_HASH
    report = dict(
        scope="사용자 요청 형상 시안; 실제 시설 채택/강도/출력/주행 검증 아님",
        units="mm (STL 자체에는 단위 메타데이터가 없으며 제작측 치수와 일치하는 해석)",
        axes="원본 좌표 유지: raw X 가로, raw Y 세로, raw Z 앞뒤",
        source_sha256=OLD_HASH, newest_official_sha256=NEW_HASH,
        output_file=output.name, output_sha256=sha(output.read_bytes()),
        road_width_mm=dict(old=450, concept=610),
        outer_size_mm=dict(old=size(old).tolist(), concept=size(proposed).tolist()),
        crossbar_length_mm=dict(old=600, concept=760),
        clear_post_span_mm=dict(old=570, concept=730),
        post_center_span_mm=dict(old=585, concept=745),
        moved_each_side_mm=80,
        unchanged_module_triangles=int(module.sum()),
        changed_triangles=int(changed.sum()), stretched_crossbar_triangles=int(stretched.sum()),
        stretch_height_range_from_base_mm=[450, 465],
        topology=topology(proposed),
        checks=dict(source_files_unchanged=True, module_records_byte_identical=True,
                    all_height_depth_coordinates_identical=True, only_x_delta_minus80_zero_plus80=True,
                    stretched_faces_only_in_top_crossbar=True, rigid_parts_edge_lengths_preserved=True,
                    triangle_topology_preserved=True, no_face_flips_or_zero_area=True,
                    output_stl_reload_identical=True))
    (HERE / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    render(old, proposed)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
