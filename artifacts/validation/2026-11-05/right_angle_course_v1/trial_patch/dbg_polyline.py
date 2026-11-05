"""[평가 전용 디버그] run 기록의 특정 시각 스캔에서 오른쪽 벽 꺾은선 후보가 어느 검사에서 거절되는지 출력한다."""
import json
import math
import sys
from pathlib import Path

import numpy as np

import arena_autonomy.local_path as lpm
import arena_autonomy.local_follow as lf
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

run, target = Path(sys.argv[1]), float(sys.argv[2])
events = [json.loads(l) for l in (run/'follow_decisions.jsonl').open(encoding='utf-8') if l.strip()]
h = events[0]['data']
follow, motion = LocalFollow(FollowConfig(**h['follow_config'])), MotionConfig(**h['motion_config'])
grab = {}
orig = lf.local_path


def spy(points, side, offset, **kw):
    if side == 'right' and 'pts' not in grab:
        grab['pts'], grab['kw'] = np.asarray(points), kw
    return orig(points, side, offset, **kw)


for e in events[1:]:
    if e['kind'] == 'reset':
        follow.reset(); continue
    if e['kind'] != 'call':
        continue
    t = e['data']['input']['reference_s']
    if abs(t-target) < .011:
        lf.local_path = spy
    replay_call(follow, e['data']['input'], motion)
    lf.local_path = orig
    if 'pts' in grab:
        break

# local_path의 구간 선택을 그대로 따라가며 samples를 얻는다(원 함수 내부 지역 변수 재현).
src = Path(lpm.__file__).read_text(encoding='utf-8')
pts = grab['pts']
calls = {}
orig_poly = lpm._polyline_offset_path


def poly_spy(samples, sign, offset, **kw):
    calls['samples'] = np.asarray(samples)
    return orig_poly(samples, sign, offset, **kw)


lpm._polyline_offset_path = poly_spy
path, info = lpm.local_path(pts, 'right', .425, **grab['kw'])
print('local_path', info.get('reason'), info.get('shape_model'))
s = calls.get('samples')
if s is None:
    print('polyline not reached'); sys.exit()
from shapely.geometry import LineString
simple = np.asarray(LineString(s).simplify(.02, preserve_topology=False).coords)
print('samples', len(s), 'first', s[0].round(3), 'last', s[-1].round(3))
print('dp vertices', len(simple)); print(simple.round(3))
seg = np.diff(simple, axis=0); L = np.linalg.norm(seg, axis=1)
for i in range(len(seg)-1):
    a, b = seg[i], seg[i+1]
    print('turn', i, round(math.degrees(math.atan2(a[0]*b[1]-a[1]*b[0], a @ b)), 1), 'len', round(L[i], 3), round(L[i+1], 3))
