"""공식 2×5 광원의 칸별 영상 위치. 상향 피치별 수직 포함 조건만 계산한다. 장착 변경·인식 합격이 아니다."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official'
WIDTH, HEIGHT, HFOV = 1280, 720, 75.
MOUNT = (.055, 0., .145)
ROAD_TOP = .003
MASK_FRACTION = .58  # 현재 검출기가 보는 영상 상단 비율


def main():
    model = ET.parse(WORLD/'world.sdf').find("./world/model[@name='it_arena_track_static']")
    scene = json.loads((WORLD/'scene.json').read_text(encoding='utf-8'))
    lamps = [tuple(map(float, model.findtext(f"link[@name='{c['name']}']/pose").split()[:3])) for c in scene['traffic_light']['lamp_channels']]
    focal = WIDTH/2/math.tan(math.radians(HFOV/2))
    half_v = math.degrees(math.atan(HEIGHT/2/focal))
    rows = []
    for slot in sorted(scene['starting_grid']['slots'], key=lambda item: -item['s_m']):
        c, s = math.cos(slot['yaw_rad']), math.sin(slot['yaw_rad'])
        cam = (slot['x']+c*MOUNT[0], slot['y']+s*MOUNT[0], ROAD_TOP+MOUNT[2])
        elevations, lateral = [], []
        for lamp in lamps:
            dx, dy, dz = lamp[0]-cam[0], lamp[1]-cam[1], lamp[2]-cam[2]
            forward, left = c*dx+s*dy, -s*dx+c*dy
            elevations.append(math.degrees(math.atan2(dz, forward)))
            lateral.append(math.degrees(math.atan2(left, forward)))
        row = dict(grid=slot['index'], s_m=slot['s_m'], elevation_deg=[round(min(elevations), 1), round(max(elevations), 1)],
                   bearing_deg=[round(min(lateral), 1), round(max(lateral), 1)], pitch={})
        for pitch in (0, 5, 10, 15, 20, 25):
            pixels = [HEIGHT/2-focal*math.tan(math.radians(e-pitch)) for e in elevations]
            row['pitch'][str(pitch)] = dict(v_px=[round(min(pixels)), round(max(pixels))],
                                            inside_image=min(pixels) >= 0 and max(pixels) < HEIGHT,
                                            above_detector_mask=max(pixels) < HEIGHT*MASK_FRACTION)
        rows.append(row)
    result = dict(scope='수평 75°·1280×720 핀홀, 현 임시 장착 높이. 피치 값은 계산이며 차량 설정이 아니다. 렌즈 왜곡·720p 크롭·실물 미측정.',
                  half_vertical_fov_deg=round(half_v, 1), detector_mask_fraction=MASK_FRACTION, rows=rows,
                  every_slot_usable_at_pitch=[p for p in ('0', '5', '10', '15', '20', '25')
                                              if all(r['pitch'][p]['inside_image'] and r['pitch'][p]['above_detector_mask'] for r in rows)])
    (Path(__file__).parent/'camera_pitch_table.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    for row in rows:
        print(row['grid'], row['elevation_deg'], row['bearing_deg'], {p: (v['inside_image'], v['above_detector_mask']) for p, v in row['pitch'].items()})
    print(result['half_vertical_fov_deg'], result['every_slot_usable_at_pitch'])


if __name__ == '__main__':
    main()
