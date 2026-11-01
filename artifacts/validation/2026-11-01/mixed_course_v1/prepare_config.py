"""짧은 추종 시험과 같은 차로, 앞차 정지·재출발 및 왼쪽 옆차가 있는 한 랩을 준비한다."""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
REF = ROOT/'config/tests/g1_traffic_full_course_v1.json'
PAIR = BASE.parent/'race_follow_pair_v1/stop_restart.json'
PRIOR = ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build():
    full = json.loads(REF.read_text(encoding='utf-8'))
    pair = json.loads(PAIR.read_text(encoding='utf-8'))
    old = json.loads((PRIOR/'traffic_config.json').read_text(encoding='utf-8'))
    case = next(x for x in full['cases'] if x['name']=='stop_restart_full_course')
    if (old['cases'] != [case] or case['duration_s']!=155 or old['actor']!=pair['actor']
            or full['acceptance']!=old['acceptance']):
        raise ValueError('앞차·각본·원래 판정과 10/31 비교 조건이 다름')
    full.update(cases=[case],registered_at='2026-11-01 대표 교통 재개',
                scope='0.35 m/s 각본 앞차의 정지·재출발과 잔디 쪽 옆차. 방지턱 없는 단일 155초 창.',
                launch={**pair['launch'],'temporary_traffic_light_trigger':'2×5 실제 픽셀 출발; 시험 녹색은 출발 신호 아님.'},
                vehicle_and_sensor_unchanged=pair['vehicle_and_sensor_unchanged'],
                matrix={'g1g2':{'cases':[case['name']],'seeds':[0]}},
                parent_config={'path':REF.relative_to(ROOT).as_posix(),'sha256':sha(REF)})
    full['launch']['speed_profile_note']='실제 등속/정지 앞차 비교에서 사용한 1.5 m/s. 단독 10 km/h 설정은 사용하지 않는다.'
    # 장시간 각본에서 기대하는 랩/바퀴 수는 원래 값이다. 짧은 추종의 판정으로 대체하지 않는다.
    return full


if __name__=='__main__':
    with (BASE/'course.json').open('x',encoding='utf-8') as f:
        json.dump(build(),f,indent=2,ensure_ascii=False)
