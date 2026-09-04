"""팀 논의용 센서 배치 기하도. 센서/차체/월드 설정을 변경하지 않습니다.

Pillow로 PNG를 만들고 같은 도형을 SVG로 함께 기록합니다.
모든 수치는 문서에 명시한 이상적 기하 비교이며 Gazebo/실차 검증이 아닙니다.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/sensors/figures/2026-09-03-placement"
INK = "#172b3a"
MUTED = "#516573"
BLUE = "#2378ba"
GREEN = "#18866f"
RED = "#b34242"
ORANGE = "#c27716"
GRAY = "#dbe3e8"
LIGHT = "#f2f5f7"


class Figure:
    def __init__(self, width: int, height: int, title: str, font: Path):
        self.width, self.height = width, height
        self.image = Image.new("RGB", (width, height), "white")
        self.font = font
        self.svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}" role="img">',
            f"<title>{html.escape(title)}</title>",
            f'<rect width="{width}" height="{height}" fill="white"/>',
        ]

    def polygon(self, points, fill, alpha=1.0, stroke=None, width=2):
        layer = Image.new("RGBA", self.image.size)
        draw = ImageDraw.Draw(layer)
        rgb = tuple(int(fill[i:i + 2], 16) for i in (1, 3, 5))
        draw.polygon(points, fill=(*rgb, round(alpha * 255)))
        if stroke:
            draw.line([*points, points[0]], fill=stroke, width=width, joint="curve")
        self.image.paste(layer, mask=layer.getchannel("A"))
        xy = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        edge = f' stroke="{stroke}" stroke-width="{width}"' if stroke else ""
        self.svg.append(f'<polygon points="{xy}" fill="{fill}" fill-opacity="{alpha}"{edge}/>')

    def rect(self, xy, fill, stroke=None, width=2, alpha=1.0):
        x0, y0, x1, y1 = xy
        self.polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], fill, alpha, stroke, width)

    def line(self, points, color=INK, width=2, dash=False):
        draw = ImageDraw.Draw(self.image)
        if dash:
            for a, b in zip(points, points[1:]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                length = math.hypot(dx, dy)
                for start in range(0, math.ceil(length), 13):
                    end = min(start + 7, length)
                    p = (a[0] + dx * start / length, a[1] + dy * start / length)
                    q = (a[0] + dx * end / length, a[1] + dy * end / length)
                    draw.line([p, q], fill=color, width=width)
        else:
            draw.line(points, fill=color, width=width, joint="curve")
        xy = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        extra = ' stroke-dasharray="7 6"' if dash else ""
        self.svg.append(f'<polyline points="{xy}" fill="none" stroke="{color}" stroke-width="{width}"{extra}/>')

    def circle(self, center, radius, fill):
        x, y = center
        ImageDraw.Draw(self.image).ellipse((x-radius, y-radius, x+radius, y+radius), fill=fill)
        self.svg.append(f'<circle cx="{x}" cy="{y}" r="{radius}" fill="{fill}"/>')

    def text(self, x, y, text, size=24, color=INK, anchor="left"):
        font = ImageFont.truetype(str(self.font), size)
        draw = ImageDraw.Draw(self.image)
        length = draw.textlength(text, font=font)
        if anchor == "middle":
            x -= length / 2
        elif anchor == "right":
            x -= length
        bbox = draw.textbbox((x, y), text, font=font, anchor="lt")
        if bbox[0] < 0 or bbox[2] > self.width or bbox[1] < 0 or bbox[3] > self.height:
            raise ValueError(f"Text outside figure: {text!r}: {bbox}")
        draw.text((x, y), text, font=font, fill=color, anchor="lt")
        self.svg.append(
            f'<text x="{x:.2f}" y="{y:.2f}" fill="{color}" '
            f'font-family="Malgun Gothic, Noto Sans CJK KR, sans-serif" '
            f'font-size="{size}" dominant-baseline="text-before-edge">{html.escape(text)}</text>'
        )

    def arrow(self, a, b, color=INK, width=2, both=False):
        self.line([a, b], color, width)
        for start, end in ([(a, b), (b, a)] if both else [(a, b)]):
            angle = math.atan2(end[1]-start[1], end[0]-start[0])
            head = [end]
            for delta in (-0.45, 0.45):
                head.append((end[0]-11*math.cos(angle+delta), end[1]-11*math.sin(angle+delta)))
            self.polygon(head, color)

    def save(self, stem):
        OUT.mkdir(parents=True, exist_ok=True)
        self.image.save(OUT / f"{stem}.png")
        (OUT / f"{stem}.svg").write_text("\n".join([*self.svg, "</svg>", ""]), encoding="utf-8")


def top_view(font):
    f = Figure(1600, 1000, "ToF 두 개의 측면·후측면 배치 비교", font)
    f.text(45, 36, "ToF 2개: 옆을 볼 것인가, 후측면을 볼 것인가", 38)
    f.text(45, 93, "공통 가정: 20×15 cm 차량 · 카메라는 뒤쪽에서 전방 주시 · ToF는 앞쪽 좌우 모서리", 24, MUTED)
    for x, color, name in [(45, BLUE, "깊이 카메라 87°"), (365, GREEN, "ToF 한 개당 60°"), (680, GRAY, "깊이 하한 안쪽"), (995, RED, "직접 관측되지 않는 뒤쪽")]:
        f.rect((x, 143, x+20, 161), color)
        f.text(x+30, 141, name, 22)
    headings = [(65, "A. 전·측면 우선", "끼어들기·앞쪽 측벽 우선", "뒤쪽 사각이 큼"),
                (100, "B. 정측면 절충", "정측면 + 비스듬한 뒤쪽", "전방 사각은 카메라와 함께 검증"),
                (120, "C. 사이드미러형", "후측면 접근을 먼저 봄", "바로 옆은 시야 끝 · 정후방도 남음")]
    scale = 500
    for index, (yaw, title, line1, line2) in enumerate(headings):
        cx, cy = 275 + index*525, 585
        left = 40 + index*525
        if index:
            f.line([(left-20, 207), (left-20, 892)], GRAY)
        f.text(left, 207, title, 29)
        f.text(left, 253, f"좌우 yaw ±{yaw}°", 24, MUTED)
        f.text(cx, 302, "전방 ↑", 23, anchor="middle")
        cam = (cx, cy+0.08*scale)
        tan_h = math.tan(math.radians(43.5))
        near, far = 0.195*scale, 0.45*scale
        f.polygon([cam, (cam[0]-near*tan_h, cam[1]-near), (cam[0]+near*tan_h, cam[1]-near)], GRAY, 0.48)
        f.polygon([(cam[0]-near*tan_h, cam[1]-near), (cam[0]-far*tan_h, cam[1]-far),
                   (cam[0]+far*tan_h, cam[1]-far), (cam[0]+near*tan_h, cam[1]-near)], BLUE, 0.12)
        f.line([(cam[0]-near*tan_h, cam[1]-near), (cam[0]+near*tan_h, cam[1]-near)], BLUE, 2, True)
        for sign in (1, -1):
            origin = (cx-sign*.075*scale, cy-.07*scale)
            angles = [math.radians(sign*yaw + a) for a in range(-30, 31)]
            sector = [origin] + [(origin[0]-.35*scale*math.sin(a), origin[1]-.35*scale*math.cos(a)) for a in angles]
            f.polygon(sector, GREEN, 0.18)
            axis = math.radians(sign*yaw)
            tip = (origin[0]-.26*scale*math.sin(axis), origin[1]-.26*scale*math.cos(axis))
            f.arrow(origin, tip, GREEN, 3)
        f.rect((cx-37.5, cy-50, cx+37.5, cy+50), "#edf1f4", INK)
        for sign in (-1, 1):
            for dy in (-36.25, 36.25):
                f.rect((cx+sign*33.75-3, cy+dy-12.5, cx+sign*33.75+3, cy+dy+12.5), INK)
        f.arrow((cx, cy+10), (cx, cy-28), INK)
        f.rect((cx-22.5, cam[1]-5, cx+22.5, cam[1]+5), BLUE)
        f.circle((cx-14, cam[1]), 3, "#ffffff")
        f.circle((cx+14, cam[1]), 3, "#ffffff")
        for sign in (-1, 1):
            f.circle((cx+sign*37.5, cy-35), 7, GREEN)
        f.line([(cx, cy+58), (cx, 764)], RED, 2, True)
        f.text(cx, 777, "정후방 미관측", 22, RED, "middle")
        f.text(left, 836, line1, 22)
        f.text(left, 872, line2, 22, MUTED)
    f.text(45, 934, "명목 수평 시야만 표시. 양안 중첩·표적 높이·차체/바퀴 가림·반사율에 따른 실제 사각은 별도입니다.", 21, MUTED)
    f.text(45, 968, "각도는 비교 후보이며 제작 확정값이 아닙니다. 회전만으로 시야 총량이 늘거나 정후방까지 보이는 것은 아닙니다.", 19, MUTED)
    f.save("tof-two-orientations")


def ground_limit(setback_cm, camera_height_cm, hood_height_cm):
    min_z = 19.5
    min_depth = min_z-setback_cm
    vertical = camera_height_cm/math.tan(math.radians(29))-setback_cm
    hood = setback_cm*hood_height_cm/(camera_height_cm-hood_height_cm)
    return {"setback_cm": setback_cm, "camera_height_cm": camera_height_cm,
            "nose_height_cm": hood_height_cm, "depth_axis_limit_cm": min_depth,
            "vertical_fov_ground_limit_cm": vertical, "hood_ground_limit_cm": hood,
            "first_ideal_ground_cm": max(0, min_depth, vertical, hood)}


def camera_side(font):
    f = Figure(1600, 1100, "전후 장착 위치와 차체 형상에 따른 깊이·바닥 사각", font)
    f.text(45, 30, "뒤로 옮기면 Min-Z 사각은 줄지만, 차체 가림을 같이 봐야 합니다", 34)
    f.text(45, 84, "수평 카메라 · 깊이 Min-Z 19.5 cm · 수직 시야 58° · 가상 차체의 중앙 단면 (바퀴 가림 제외)", 23, MUTED)
    rows = [(4.5, 7.5, 3, "A. 현재 카메라 위치에 가까운 전방안", "전방 깊이 하한이 주된 제한"),
            (18, 12, 8, "B. 후방 + 높은 평평한 상판", "차체가 근거리 바닥을 가림"),
            (18, 12, 3, "C. 후방 + 앞이 낮은 경사형 차체", "차체 가림을 줄이는 유효한 대안")]
    checks = []
    for i, (setback, height, hood, title, note) in enumerate(rows):
        baseline, scale, xzero = 371+i*282, 11.6, 325
        def point(x, z):
            return (xzero+x*scale, baseline-z*scale)
        values = ground_limit(setback, height, hood)
        checks.append(values)
        f.text(45, 134+i*282, title, 26)
        f.line([point(-23, 0), point(49, 0)], INK, 2)
        for cm in (0, 10, 20, 30, 40):
            x, y = point(cm, 0)
            f.line([(x, y-4), (x, y+5)], INK)
            f.text(x, y+13, f"{cm}", 18, MUTED, "middle")
        cam = point(-setback, height)
        minx = values["depth_axis_limit_cm"]
        f.rect((*point(minx, 16), *point(49, 0)), BLUE, alpha=0.055)
        f.line([point(minx, 0), point(minx, 16)], BLUE, 2, True)
        f.line([cam, point(49, height)], BLUE, 2, True)
        f.text(point(minx, 0)[0]+8, baseline-190, f"깊이 시작 {minx:.1f} cm", 20, BLUE)
        fov_end = values["vertical_fov_ground_limit_cm"]
        f.line([cam, point(fov_end, 0)], GRAY, 2, True)
        top_rear = 10 if i == 2 else hood
        outline = [point(-20, 2), point(-20, top_rear), point(0, hood), point(0, 2)]
        f.polygon(outline, "#dbe3e8", stroke=INK)
        for wx in (-17.25, -2.75):
            f.circle(point(wx, 2.5), 2.5*scale, "#b8c4cb")
            f.circle(point(wx, 2.5), 1.2*scale, "#eef2f4")
        hood_end = values["hood_ground_limit_cm"]
        f.line([cam, point(hood_end, 0)], ORANGE, 3)
        f.line([point(0, hood), point(0, 0)], ORANGE, 3)
        f.line([point(-setback, 4), cam], INK, 4)
        f.rect((cam[0]-13, cam[1]-8, cam[0]+13, cam[1]+8), BLUE)
        f.circle((cam[0]+10, cam[1]), 3, "#ffffff")
        visible = values["first_ideal_ground_cm"]
        f.line([point(0, -.4), point(visible, -.4)], RED, 5)
        f.line([point(visible, -.4), point(49, -.4)], GREEN, 5)
        f.circle(point(visible, 0), 5, GREEN)
        f.text(945, 194+i*282, f"카메라: 앞범퍼 뒤 {setback:g} cm / 높이 {height:g} cm", 24)
        f.text(945, 238+i*282, f"차체 앞끝 높이: {hood:g} cm (비교 가정)", 23, MUTED)
        f.text(945, 282+i*282, f"첫 바닥 관측: 범퍼 앞 약 {visible:.1f} cm", 25, GREEN)
        f.text(945, 326+i*282, note, 23)
    f.text(45, 1014, "파랑 점선: 광축 방향 깊이 하한  |  주황: 차체 앞끝을 스치는 시선  |  빨강/초록 바닥선: 미관측/기하상 관측", 20, MUTED)
    f.text(45, 1050, "축 눈금: 앞범퍼 기준 cm. 차체 형상·높이는 임시 예시이며, 표적 차량의 검출거리·제동거리·실물 성능을 뜻하지 않습니다.", 19, MUTED)
    f.save("camera-position-and-hood")
    return checks


def wall_road(font):
    f = Figure(1600, 860, "벽으로 도로 경계를 추정하되 가린 점유는 미확인", font)
    f.text(45, 34, "도로 경계는 벽으로 추정할 수 있습니다. 가려진 장애물까지 알 수 있는 것은 아닙니다", 32)
    f.text(45, 90, "현재 공식 v2026.09.02의 일반 본선: 도로 45 cm + 좌우 잔디 각 20 cm", 25, MUTED)
    f.rect((65, 220, 930, 245), "#6d7c87")
    f.rect((65, 245, 930, 345), "#c5dfc1")
    f.rect((65, 345, 930, 570), "#eef1f3")
    f.rect((65, 570, 930, 670), "#c5dfc1")
    f.rect((65, 670, 930, 695), "#6d7c87")
    f.line([(65, 345), (930, 345)], BLUE, 3, True)
    f.line([(65, 570), (930, 570)], BLUE, 3, True)
    f.line([(65, 457.5), (930, 457.5)], GRAY, 2, True)
    f.text(79, 258, "잔디", 23)
    f.text(79, 582, "잔디", 23)
    f.text(79, 180, "벽 안쪽 면", 23, MUTED)
    f.arrow((157, 210), (157, 245), MUTED)
    shadow_top = 457.5 + (420-457.5)*(930-218)/(483-218)
    shadow_bottom = 457.5 + (495-457.5)*(930-218)/(483-218)
    f.polygon([(483, 420), (930, shadow_top), (930, shadow_bottom), (483, 495)], RED, .095)
    f.line([(218, 457.5), (483, 420), (930, shadow_top)], RED, 2, True)
    f.line([(218, 457.5), (483, 495), (930, shadow_bottom)], RED, 2, True)
    f.rect((205, 420, 305, 495), "#d1e7f5", BLUE, 2)
    f.arrow((230, 457.5), (286, 457.5), BLUE, 3)
    f.rect((209, 434, 218, 481), BLUE)
    f.rect((483, 420, 583, 495), "#e4c5a4", ORANGE, 2)
    f.text(255, 525, "우리 차량", 23, anchor="middle")
    f.text(533, 525, "앞차", 23, anchor="middle")
    f.text(757, 418, "경로 모양은 추정 가능", 22, anchor="middle")
    f.text(757, 464, "새 장애물 유무는 미확인", 22, RED, "middle")
    for y0, y1, label in [(245, 345, "20 cm"), (345, 570, "45 cm"), (570, 670, "20 cm")]:
        f.arrow((965, y0), (965, y1), MUTED, 2, True)
        f.text(990, (y0+y1)/2-12, label, 23)
    f.text(1110, 232, "관측과 추정을 분리", 28)
    f.circle((1120, 301), 7, "#6d7c87")
    f.text(1142, 285, "벽: 직접 관측", 24)
    f.line([(1108, 363), (1134, 363)], BLUE, 3, True)
    f.text(1142, 345, "도로 경계: 벽에서 추정", 23)
    f.rect((1112, 417, 1128, 433), RED, alpha=.2)
    f.text(1142, 405, "앞차 뒤: 점유 미확인", 24)
    f.text(1110, 500, "일반 본선 중심선", 23, MUTED)
    f.text(1110, 540, "벽 안쪽 면에서 약 42.5 cm", 24)
    f.text(1110, 605, "갈림길·합류부는 예외", 25, RED)
    f.text(45, 752, "출처: 공식 배포 ZIP의 scene.json 및 track_gen.py. 그림은 직선부 설명용이며 실제 트랙 전체의 배치도는 아닙니다.", 21, MUTED)
    f.text(45, 792, "가림 중에는 벽·이전 관측·엔코더/IMU로 짧은 경로를 유지하되, 보지 못한 공간을 자동으로 안전한 도로로 채우지 않습니다.", 20, MUTED)
    f.save("wall-road-and-occlusion")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--font", type=Path, default=Path("C:/Windows/Fonts/malgun.ttf"))
    args = parser.parse_args()
    if not args.font.is_file():
        parser.error("--font에 한글을 지원하는 TTF 경로를 지정하십시오.")
    top_view(args.font)
    cases = camera_side(args.font)
    wall_road(args.font)
    assert math.isclose(cases[0]["depth_axis_limit_cm"], 15.0)
    assert math.isclose(cases[1]["depth_axis_limit_cm"], 1.5)
    assert math.isclose(cases[1]["first_ideal_ground_cm"], 36.0)
    assert math.isclose(cases[2]["first_ideal_ground_cm"], 6.0)
    check = {"date": "2026-09-03", "kind": "ideal_geometry_only_not_gazebo_or_hardware_validation",
             "camera": {"model": "D435i", "profile": "848x480", "min_z_cm": 19.5,
                        "hfov_deg": 87, "vfov_deg": 58, "pitch_deg": 0},
             "tof": {"model": "VL53L7CX", "hfov_deg": 60, "yaw_cases_deg": [65, 100, 120]},
             "front_blind_limit_gain_cm": 13.5,
             "gain_at_20kmh_ms": 0.135/(20/3.6)*1000,
             "camera_side_cases": cases,
             "limitations": ["no binocular invalid band", "no sensor noise or material response",
                             "top view excludes body occlusion and target height",
                             "side view only camera midline, flat ground and assumed nose shape",
                             "no detection or safe-speed claim"]}
    (OUT / "geometry-checks.json").write_text(json.dumps(check, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    print(json.dumps(check, ensure_ascii=False, indent=2))
    print(f"Created 3 PNG + 3 SVG + geometry-checks.json in {OUT}")


if __name__ == "__main__":
    main()
