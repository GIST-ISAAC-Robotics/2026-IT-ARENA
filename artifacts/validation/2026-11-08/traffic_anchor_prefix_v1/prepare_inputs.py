"""traffic_lap_anchor_v1 사본을 44초 코너 구간 사례로 바꾼다. 실행 전 1회."""
import copy, hashlib, json
from pathlib import Path

HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'traffic_lap_anchor_v1'
CASE = 'two_actor_corner_prefix'
REL = 'artifacts/validation/2026-11-08/traffic_anchor_prefix_v1'
OLD_REL = 'artifacts/validation/2026-11-08/traffic_lap_anchor_v1'


def dump(path, data):
    path.write_bytes((json.dumps(data, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))


cfg = json.loads((OLD/'new_track_course.json').read_bytes())
assert len(cfg['cases']) == 1 and cfg['cases'][0]['name'] == 'two_actor_course_lap'
case = copy.deepcopy(cfg['cases'][0])
case.update(name=CASE, route_kind='first_curve', duration_s=44.0,
            schedule=[dict(from_s=0.0, to_s=44.0, speed_mps=0.5)],
            expected='새 코스 두 상대(0.5 m/s 중앙) 뒤 44초 코너 구간. 이전 18.98 m 정체 지점을 지나 21.5 m 이상 진행. 전체 랩 시험 아님.')
cfg['cases'] = [case]
rules = copy.deepcopy(cfg['acceptance']['two_actor_course_lap'])
del rules['minimum_laps'], rules['maximum_actor_laps']
rules.update(minimum_progress_m=21.5, required_exit_s_m=18.5, maximum_actor_s_m=27.0, minimum_heading_change_deg=90,
             prefix_basis='r22 96초 시험의 18.98 m 정체 지점을 약 2.5 m 지나는 사전 목표. 실행 전 고정, 결과 후 완화 금지.')
cfg['acceptance'][CASE] = rules
cfg['scope'] = ('새 코스 두 상대 44초 코너 구간 폐루프 1회. r23 경로 표현 전환 앵커 보완 '
                '(config/tests/corner_path_anchor_transition_r23_sha256.json). 원 r22 96초 실패 보존, 소급 성공 아님.')
cfg['registered_at'] = '2026-11-08 traffic_anchor_prefix_v1 실행 전 준비'
cfg['straight_section_check'] = ('직선/첫코너 기존 검사는 불변. 이번 사례는 first_curve 제한 경로 검사(legacy 이름)를 '
                                 '등록 코너 구간에 적용해 두 상대 각본을 1 cm마다 도로/벽 대조한다.')
old_cfg = OLD/'new_track_course.json'
cfg['derived_from'] = dict(path=f'{OLD_REL}/new_track_course.json', sha256=hashlib.sha256(old_cfg.read_bytes()).hexdigest(),
    changed_keys=['scope', 'registered_at', 'straight_section_check', 'cases', f'acceptance.{CASE}', 'derived_from'],
    controller_source_change='config/tests/corner_path_anchor_transition_r23_sha256.json')
dump(HERE/'new_track_course.json', cfg)

overlay = json.loads((OLD/'additional_actors.json').read_bytes())
overlay.update(base_case=CASE, registered_at='2026-11-08 before traffic_anchor_prefix_v1 run',
               scope='two centerline scripted actors 6m apart and single ego; 44 s registered corner prefix; no controller map input')
(extra,) = overlay['additional_actors']
extra.update(route_kind='first_curve', schedule=[dict(from_s=0.0, to_s=44.0, speed_mps=0.5)])
dump(HERE/'additional_actors.json', overlay)

for name in ('run.sh', 'run_live.sh'):
    path = HERE/name
    text = path.read_bytes().decode('utf-8')
    assert OLD_REL in text
    text = text.replace(OLD_REL, REL).replace('two_actor_course_lap', CASE)
    path.write_bytes(text.encode('utf-8'))
print('prepared', CASE)
