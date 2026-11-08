"""평가 전용 기하 조사: 두 월드 중심선의 직선 구간·출발 칸 위치를 비교한다.

제어 입력이 아니다. 결과는 재등록 계획의 참고값이며 시나리오 s좌표를 자동 변환하지 않는다.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).with_name('geometry_probe.json')


def straights(world):
    route = np.genfromtxt(world/'centerline.csv', delimiter=',', names=True)
    x, y, s = route['x_m'], route['y_m'], route['s_m']
    heading = np.unwrap(np.arctan2(np.diff(y), np.diff(x)))
    bends = np.abs(np.diff(heading)) > 1e-3
    runs, start = [], 0
    for index, bend in enumerate(bends):
        if bend:
            if s[index+1]-s[start] > 1.0:
                runs.append(dict(from_s=round(float(s[start]), 3), to_s=round(float(s[index+1]), 3),
                                 length_m=round(float(s[index+1]-s[start]), 3)))
            start = index+1
    if s[-1]-s[start] > 1.0:
        runs.append(dict(from_s=round(float(s[start]), 3), to_s=round(float(s[-1]), 3),
                         length_m=round(float(s[-1]-s[start]), 3)))
    scene = json.loads((world/'scene.json').read_text(encoding='utf-8'))
    return dict(world=world.relative_to(ROOT).as_posix(), samples=int(len(s)),
                lap_length_m=scene['track']['lap_length_m'], width_m=scene['track']['width_m'],
                slots={int(slot['index']): round(float(slot['s_m']), 3) for slot in scene['starting_grid']['slots']},
                traffic_light_s_m=scene['traffic_light'].get('s_m'),
                straights_over_1m=runs)


def main():
    result = {name: straights(ROOT/'src/arena_gazebo/worlds'/name)
              for name in ('it_arena_official', 'it_arena_official_v2026_11_06')}
    result['scope'] = 'evaluation-only geometry; not a controller input and not an automatic scenario conversion'
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
