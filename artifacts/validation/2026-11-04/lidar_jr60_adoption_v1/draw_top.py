"""geometry.json으로 JR60P20 채택 상면도 1장을 그린다. Pillow, 맑은 고딕(WSL에서는 /mnt/c 경로). 인자 없음."""
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
G = json.loads((HERE/'geometry.json').read_text(encoding='utf-8'))
FONT_DIRS = (Path('C:/Windows/Fonts'), Path('/mnt/c/Windows/Fonts'))
W, H = 1500, 900
im = Image.new('RGB', (W, H), '#f4f7f6')
d = ImageDraw.Draw(im, 'RGBA')


def font(size, bold=False):
    name = 'malgunbd.ttf' if bold else 'malgun.ttf'
    path = next(p/name for p in FONT_DIRS if (p/name).is_file())
    return ImageFont.truetype(str(path), size)


def label(x, y, text, size, ink='#173e46', bold=False, anchor='la'):
    d.text((x, y), text, fill=ink, font=font(size, bold), anchor=anchor)


CX, CY, S = 470, 500, 2.6  # 차체 중심 화면 위치, px/mm


def xy(x, y):
    return CX + S*x, CY - S*y


def circle(x, y, r, fill, outline=None, width=1):
    d.ellipse((*xy(x - r, y + r), *xy(x + r, y - r)), fill=fill, outline=outline, width=width)



sx, sy, _ = G['sensor_mm']
# 그림자(마스크 여유 포함) 쐐기: 센서에서 200 mm까지
for row in G['mask_sectors']:
    c, h = row['center_deg'], row['half_deg']
    color = (214, 120, 60, 70) if row['kind'] == 'scan_plane_wires' else (90, 110, 120, 60)
    pts = [xy(sx, sy)] + [xy(sx + 200*math.cos(math.radians(a)), sy + 200*math.sin(math.radians(a)))
                          for a in [c - h + i*(2*h)/12 for i in range(13)]]
    d.polygon(pts, fill=color)
# 정후방 연속 개방
rear = G['openings']['jr60p20_reserve_d20']['mask_with_margin']['rear_continuous_deg']
pts = [xy(sx, sy)] + [xy(sx + 200*math.cos(math.radians(a)), sy + 200*math.sin(math.radians(a)))
                      for a in [180 - rear/2 + i*rear/12 for i in range(13)]]
d.polygon(pts, fill=(60, 160, 110, 55))

# 차체 외곽 200×150, 뒤바퀴 평면 범위
d.rectangle((*xy(-100, 75), *xy(100, -75)), outline='#55787c', width=3)
wy0, wy1 = G['hardware_flags']['rear_wheel_plan_y_mm']
ax = G['hardware_flags']['rear_axle_x_mm']
for sgn in (1, -1):
    d.rectangle((*xy(ax - 32.5, sgn*wy1 if sgn > 0 else -wy0), *xy(ax + 32.5, sgn*wy0 if sgn > 0 else -wy1)),
                fill=(120, 120, 120, 40), outline='#9aa7a8', width=1)
    d.rectangle((*xy(ax + 135 - 32.5, sgn*wy1 if sgn > 0 else -wy0), *xy(ax + 135 + 32.5, sgn*wy0 if sgn > 0 else -wy1)),
                fill=(120, 120, 120, 25), outline='#c3cccd', width=1)
# C1 본체 55.6 mm 정사각과 회전축
d.rectangle((*xy(sx - 27.8, 27.8), *xy(sx + 27.8, -27.8)), fill='#dbe8f0', outline='#2b5d7a', width=2)
circle(sx, sy, 2.5, '#2b5d7a')
# 지주
for x, y, dia in G['posts_mm']:
    circle(x, y, dia/2, '#4a4f52')
# 배선 예약: Ø20 외접원 + 16×12 외곽 사각형 + 2 mm 벽 안쪽
for ch, (x, y, dia) in zip(G['channels'], G['reserve_envelopes_mm']):
    circle(x, y, dia/2, (214, 120, 60, 40), '#c0602a', 2)
    d.polygon([xy(*p) for p in ch['outer_corners_mm']], outline='#7a3412', width=2)
    d.polygon([xy(*p) for p in ch['inner_wall_corners_mm']], outline='#b0703f', width=1)
# 후면 ID49 판(평면 위치, 높이 140 mm 중심)
mx, _ = G['marker']['plan_xy_mm']
d.line((*xy(mx, 25), *xy(mx, -25)), fill='#1d6b52', width=5)

label(*xy(sx - 30, -34), 'C1 회전축 x=+60, 광축 75 mm', 15, '#2b5d7a', anchor='rt')
label(*xy(22, 70), '앞 지주 x=+20, y=±62, Ø7', 15, anchor='lb')
label(*xy(-100, -82), '200 × 150 mm 외곽', 14, '#55787c', anchor='lt')
label(*xy(-104, 0), 'ID49 중심 140 mm\n(아래끝 115 mm)', 14, '#1d6b52', anchor='rm')
label(*xy(-60, 30), '정후방 개방', 15, '#2d7a55', bold=True, anchor='mm')
label(*xy(-60, 22), f'{rear:.1f}° (마스크)', 14, '#2d7a55', anchor='mm')

# 머리글은 그림자 위에 그린다
d.rectangle((0, 0, W, 112), fill='#163e46')
label(36, 14, 'IT ARENA  /  임시 차량 JR60P20  ·  최종 CAD 전 시뮬 기준', 21, '#b4ddd4', True)
label(36, 48, '센서 +60 · 앞 지주 +20 · 뒤 지주 바깥 배선 예약 외곽 Ø20', 34, 'white', True)
# 오른쪽 설명
x0 = 900
d.rounded_rectangle((x0, 140, W - 30, H - 40), radius=14, fill='white', outline='#c8d9d7', width=2)
o = G['openings']
lines = [
    ('기하 (mm, 차체 중심, +x 전방)', True),
    ('뒤 지주 x=-80, y=±52, Ø7', False),
    ('예약 외곽 중심 (-88.906, ±55.308), Ø20', False),
    ('= 지주 + 9.5 × (센서→지주 단위벡터)', False),
    ('U채널 외곽 접선 16 × 반경 12, 벽 2', False),
    (f"사각형 모서리 최대 {G['channels'][0]['max_corner_distance_from_envelope_center_mm']:.3f} ≤ 10", False),
    ('', False),
    ('스캔면 가림 (물리 / 마스크 ±0.5°)', True),
    (f"정면 연속 {o['jr60p20_reserve_d20']['physical']['front_continuous_deg']:.1f}° / "
     f"{o['jr60p20_reserve_d20']['mask_with_margin']['front_continuous_deg']:.1f}°", False),
    (f"정후방 연속 {o['jr60p20_reserve_d20']['physical']['rear_continuous_deg']:.1f}° / {rear:.1f}°", False),
    (f"  얇은 2 mm 배선 기록 {o['jr60p20_thin_wire_run']['physical']['rear_continuous_deg']:.1f}°", False),
    (f"전 방향 가림 합 {o['jr60p20_reserve_d20']['physical']['blind_total_deg']:.1f}° "
     f"(얇은 배선 {o['jr60p20_thin_wire_run']['physical']['blind_total_deg']:.1f}°)", False),
    ('', False),
    ('범위', True),
    ('외곽 원기둥은 표시·광선 가림 예약이며', False),
    ('실제 U채널 CAD·강도·하네스 실측이 아니다.', False),
    ('질량·충돌·동역학 불변. 주행 성능 미측정.', False),
    ('채널 외곽 |y| 64.9 > 뒤바퀴 안쪽면 61.5:', False),
    ('바퀴 위끝 65 mm 아래로 내리면 CAD 확인.', False),
]
y = 165
for text, bold in lines:
    label(x0 + 24, y, text, 19 if bold else 17, '#173e46' if bold else '#2e4a50', bold)
    y += 34 if bold else 29
# 범례
for i, (color, text) in enumerate(((((214, 120, 60, 160)), '예약 외곽 그림자'), (((90, 110, 120, 160)), '지주 그림자'),
                                   (((60, 160, 110, 160)), '정후방 개방'))):
    d.rectangle((40 + 230*i, 845, 62 + 230*i, 867), fill=color)
    label(70 + 230*i, 856, text, 16, anchor='lm')

im.save(HERE/'jr60_adoption_top.png', optimize=True)
print(HERE/'jr60_adoption_top.png')
