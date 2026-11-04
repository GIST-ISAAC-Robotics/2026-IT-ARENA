"""C1 mount study: chassis-fixed 2-D visibility; mm, +x forward, +y left.

Run: python scripts/study_c1_mount.py [--out DIRECTORY]. No ROS or third-party dependencies.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / "artifacts/validation/2026-11-03/lidar_mount_study_v1"
STEP = 0.72
PHASES = 4
MIN_MM = 50
REGIONS = {
    "front30": [(0, 30)], "diag30_60": [(30, 60)], "oblique60_85": [(60, 85)],
    "side85_100": [(85, 100)], "rear_quarter100_150": [(100, 150)],
    "rear150_180": [(150, 180)], "front100": [(0, 100)],
}
# Mirrored right and left, unless crossing centerline. Rectangles specify point centers.
ZONES = {
    "front": (120, 1120, -75, 75, False),
    "entry": (120, 720, 100, 400, True),
    "beside": (-80, 80, 100, 300, True),
    "rear_quarter": (-720, -120, 100, 400, True),
    "rear": (-1120, -120, -75, 75, False),
}


@dataclass(frozen=True)
class Layout:
    id: str
    name: str
    sx: float = 0
    posts: tuple = ()  # (x, y, diameter), circular optical cross-sections
    plate: tuple = ()  # (x of nearest face, width), centered behind the sensor
    rear_mask: float = 0  # artificial full angle, no simulated material
    note: str = ""


def four(front=80, rear=-80, y=62, diameter=7):
    return tuple((x, s*y, diameter) for x in (front, rear) for s in (-1, 1))


def build():
    rp = four()[2:]
    return [
        Layout("O", "스캔면 개방", note="상판 지지·배선을 광학창 위나 아래로"),
        Layout("R", "현행 후방 60°", 60, rear_mask=60, note="실행 프로필의 임시 마스크"),
        Layout("A", "중앙 · 네 모서리", posts=four(), note="중앙 센서와 네 모서리 지지"),
        Layout("B", "센서 30 mm 후퇴", -30, four(), note="같은 기둥, 앞 그림자만 더 좁게"),
        Layout("C", "센서 30 mm 전진", 30, four(), note="같은 기둥, 앞 그림자가 더 옆으로"),
        Layout("D", "앞 기둥만 후퇴", 0, four(35), note="센서 중앙, 앞 기둥 x=+35"),
        Layout("E", "센서 앞 · 지지 뒤", 40, four(15), note="센서 +40, 앞 기둥 +15, 앞판 돌출 85 mm"),
        Layout("E2", "짧은 상판 · 네 기둥", 45, four(10), note="센서 +45, 앞 기둥 +10, 짧은 상판 돌출 25 mm"),
        Layout("F", "뒤 기둥 두 개", 0, rp, note="뒤 두 지지만 측정 높이 통과"),
        Layout("G", "기둥에 붙인 배선", posts=four()[:3]+((-80,62,9),), note="A의 뒤왼쪽 묶음 유효 지름 7→9 mm"),
        Layout("H", "별도 배선 한 줄", posts=four()+((42,16,5),), note="A에 따로 지난 Ø5 묶음 1개 추가"),
        Layout("J", "넓은 후방 적층", plate=(-60,100), note="같은 높이에서 폭 100, 앞면까지 60 mm"),
        Layout("K", "가까운 후방 적층", plate=(-45,140), note="같은 높이에서 폭 140, 앞면까지 45 mm"),
        Layout("L", "현행 위치 · 네 모서리", 60, four(), note="R과 같은 센서 위치, 후방 마스크를 네 기둥으로 교체"),
        Layout("O60", "현행 위치 · 개방", 60, note="R·L의 위치 대조군, 앞범퍼까지 40 mm"),
    ]


def intervals(center, half):
    return [(max(-180, center+shift-half), min(180, center+shift+half))
            for shift in (-360, 0, 360) if max(-180, center+shift-half) < min(180, center+shift+half)]


def union(arcs):
    out = []
    for a, b in sorted(arcs):
        if out and a <= out[-1][1]:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def mask(layout):
    arcs = []
    for x, y, d in layout.posts:
        dx = x-layout.sx
        arcs += intervals(math.degrees(math.atan2(y, dx)), math.degrees(math.asin(d/2/math.hypot(dx,y))))
    if layout.rear_mask:
        arcs += intervals(180, layout.rear_mask/2)
    if layout.plate:
        x, w = layout.plate
        assert x < layout.sx
        arcs += intervals(180, math.degrees(math.atan2(w/2, layout.sx-x)))
    return union(arcs)


def overlap(arcs, sectors):
    return sum(max(0, min(b, d) - max(a, c)) for a, b in arcs for c, d in sectors)


def inside(ang, arcs):
    return any(a <= ang <= b for a, b in arcs)


def centers(zone, pitch, margin=0):
    x1, x2, y1, y2, mirror = ZONES[zone]
    for x in range(x1+pitch//2, x2, pitch):
        for y in range(y1+pitch//2, y2, pitch):
            # keep every finite circular target outside the chassis rectangle
            if math.hypot(max(abs(x)-100, 0), max(abs(y)-75, 0)) < margin:
                continue
            yield x, y
            if mirror:
                yield x, -y


def point_stats(layout, arcs):
    out = {}
    for name in ZONES:
        points = list(centers(name, 10))
        shadow = short = 0
        for x,y in points:
            dx=x-layout.sx
            if math.hypot(dx,y) < MIN_MM:
                short += 1
            elif inside(math.degrees(math.atan2(y,dx)), arcs):
                shadow += 1
        out[name] = dict(n=len(points), shadow=shadow, too_close=short,
                         hidden_pct=round(100*(shadow+short)/len(points), 3))
    return out


def finite(layout, arcs, diam, zone):
    # Deterministic four quarter-step offsets of the 500-ray scan; a static circle
    # at the scan plane, not a claim about vehicle outline or real clustering.
    # If the first surface is inside minimum range, we do not see through it.
    angles = [-180+(i+phase/PHASES)*STEP for phase in range(PHASES) for i in range(500)]
    clear = [not inside(a, arcs) for a in angles]
    zero = few = n = minhits = 0
    minhits = 500
    for x,y in centers(zone, 50, diam/2):
        dx, dy = x-layout.sx,y
        dist = math.hypot(dx,dy)
        bearing = math.degrees(math.atan2(dy, dx))
        half = math.degrees(math.asin(diam/(2*dist)))
        for p in range(PHASES):
            k0 = math.ceil((bearing-half+180)/STEP-p/PHASES)
            k1 = math.floor((bearing+half+180)/STEP-p/PHASES)
            hits = 0
            for j in range(k0,k1+1):
                k = p*500+j%500
                if clear[k]:
                    if dist-diam/2 >= MIN_MM:
                        hits += 1
                    else:
                        da = math.radians(angles[k]-bearing)
                        entry = dist*math.cos(da)-math.sqrt(max(0,(diam/2)**2-(dist*math.sin(da))**2))
                        hits += entry >= MIN_MM
            zero += hits == 0
            few += hits < 3
            n += 1
            minhits = min(hits, minhits)
    return dict(samples=n, zero=zero, under3=few, min_hits=minhits, zero_pct=round(100*zero/n,3))


def wall(layout, arcs, side):
    # Straight parallel wall from -500 to +1000 mm; 5 mm midpoint bins.
    masked = longest = run = 0
    for i in range(300):
        x = -500+(i+.5)*5
        hit = inside(math.degrees(math.atan2(side*225,x-layout.sx)),arcs)
        masked += hit*5
        run = run+5 if hit else 0
        longest = max(longest,run)
    return dict(missing_mm=masked,longest_missing_mm=longest)


def analysis(layout):
    arcs = mask(layout)
    sector = {key:round(overlap(arcs, values+[(-b,-a) for a,b in values]),4) for key,values in REGIONS.items()}
    return dict(**asdict(layout), arcs=arcs, total_deg=round(sum(b-a for a,b in arcs),4),
                sector_deg=sector, points=point_stats(layout,arcs),
                targets={str(d):{k:finite(layout,arcs,d,k) for k in ZONES} for d in (50,150)},
                walls={k:wall(layout,arcs,s) for k,s in [('left',1),('right',-1)]})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    args.out.mkdir(exist_ok=True,parents=True)
    data=[analysis(layout) for layout in build()]
    (args.out/"results.json").write_text(json.dumps(dict(unit="mm",roi=ZONES,sample_step_deg=STEP,
        scan_phases=PHASES,minimum_range_mm=MIN_MM,layouts=data),ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    with (args.out/"sweep.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f)
        w.writerow(["sensor_x_mm","front_post_x_mm","abs_y_mm","diameter_mm","total_blind_deg",
                    "inside_plus_minus_100_deg","front_post_center_deg","front_post_nearest_edge_deg"])
        for sx in (-40,-30,-15,0,15,30,35,40,45,60):
            for fx in (10,15,20,25,35,50,65,80):
                for y in (52,57,62,67):
                    for d in (4,6,8,10):
                        l=Layout("","",sx,four(fx,-80,y,d))
                        arcs=mask(l)
                        c=math.degrees(math.atan2(y,fx-sx))
                        h=math.degrees(math.asin(d/2/math.hypot(y,fx-sx)))
                        w.writerow([sx,fx,y,d,round(sum(b-a for a,b in arcs),4),
                                    round(overlap(arcs,[(-100,100)]),4),round(c,4),round(c-h,4)])
    with (args.out/"rear_width.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f); w.writerow(["width_mm","distance_to_face_mm","total_blind_deg"])
        for width in (5,10,25,50,75,100,125,140):
            for dist in (35,45,60,75,90,110,140):
                w.writerow([width,dist,round(2*math.degrees(math.atan2(width/2,dist)),4)])
    for r in data:
        print(r["id"], r["name"], "total", r["total_deg"], "front100", r["sector_deg"]["front100"],
              "front/entry/side/rearquarter/rear hidden%",[v["hidden_pct"] for v in r["points"].values()],
              "small zero", [v["zero"] for v in r["targets"]["50"].values()])


if __name__ == "__main__":
    main()
