"""빠른 앞차가 직선에서 급정지하는 등록 각본을 기존 정지·재출발 설정에서 파생한다.

후보는 빠른 앞차에 더 다가가므로 기존 0.35 m/s 정지 각본만으로는 접근 중 급정지를 재지 못한다.
직선 3.5 m 안에 맞춰 앞차 속도와 시점만 바꾼다. 상대 형상·마진·접촉 판정선은 그대로다.
"""
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
PARENT = BASE.parent / 'race_follow_pair_v1/stop_restart.json'
ROLLING = BASE.parent / 'rolling_lead_v1/course.json'
OUTPUT = BASE / 'fast_stop.json'

config = json.loads(PARENT.read_bytes())
rolling = json.loads(ROLLING.read_bytes())
config['scope'] = '앞차 0.85 m/s 접근 중 급정지·재출발. 후보와 기준을 같은 입력 조건으로 비교한다.'
config['parent'] = dict(path=PARENT.relative_to(BASE.parents[3]).as_posix())
config['cases'] = [dict(name='stop_restart', duration_s=9.85, schedule=[
    dict(from_s=0., to_s=3.6, speed_mps=.85), dict(from_s=3.6, to_s=8.6, speed_mps=0.),
    dict(from_s=8.6, to_s=9.85, speed_mps=.35)],
    expected='follow a fast lead, stop or bypass without contact when it stops, then continue')]
config['acceptance']['stop_restart'] = dict(config['acceptance']['stop_restart'], hold_window_s=[3.6, 8.6],
    basis='기존 정지·재출발 판정값 유지, 정지 구간과 재출발 뒤 거리만 이 각본 시점에 맞춤. 재출발 뒤 앞차 이동은 0.44 m.')
config['acceptance']['harness'] = rolling['acceptance']['harness']
config['matrix'] = dict(g1g2=dict(cases=['stop_restart'], seeds=[0], reserve=['closing', 'low_motion']))
with OUTPUT.open('x', encoding='utf-8', newline='\n') as f:
    json.dump(config, f, ensure_ascii=False, indent=2)
print(json.dumps(config['acceptance']['harness'], ensure_ascii=True)[:300])
