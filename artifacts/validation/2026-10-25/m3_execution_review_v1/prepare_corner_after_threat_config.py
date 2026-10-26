"""run27 뒤 첫 코너 연결 시험의 새 config/overlay 판본 생성. 기존 config/overlay·판정선은 수정하지 않는다."""
import copy
import hashlib
import json
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
PARENT = 'config/tests/g1_traffic_extensions_v1.json'
STOP = 'config/tests/g1_traffic_v1.json'
OVERLAY_PARENT = 'config/tests/e3_stop_restart_v1.json'
CONFIG = 'config/tests/g1_corner_after_threat_v1.json'
OVERLAY = 'config/tests/e3_stop_restart_corner_v1.json'
CASE = 'stop_restart_first_curve'
DURATION = 40.0


def sha(path):
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def main():
    for out in (CONFIG, OVERLAY):
        if (ROOT/out).exists():
            raise FileExistsError(out)
    parent = json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
    stop = json.loads((ROOT/STOP).read_text(encoding='utf-8'))
    first = next(c for c in parent['cases'] if c['name'] == 'first_curve')
    base = next(c for c in stop['cases'] if c['name'] == 'stop_restart')
    cfg = copy.deepcopy(parent)
    cfg['registered_at'] = '2026-10-25 before run32 execution (GPT design choice (a), 09:51 KST)'
    cfg['scope'] = ('run27 정지·재개 뒤 해제되지 않은 M3가 첫 U자 코너로 진행하는지의 교착 확인/반증 1회. '
                    '앞차는 stop_restart 0–4 s 주행·4–9 s 정지 뒤 first_curve 도로 내 경로를 0.35 m/s로 계속 간다. '
                    '통과 유도·대회 성공 판정이 아니다. 평가기 내장 gate는 40 s 창 전체로 계산되므로 정지·재개 판정은 '
                    '사후 15 s 창, 코너 판정은 그 뒤 창으로 따로 읽는다.')
    cfg['parent_config'] = {'path': PARENT, 'sha256': sha(PARENT), 'stop_restart_source': STOP, 'stop_restart_sha256': sha(STOP)}
    assert base['schedule'][:2] == [{'from_s': 0.0, 'to_s': 4.0, 'speed_mps': 0.35}, {'from_s': 4.0, 'to_s': 9.0, 'speed_mps': 0.0}]
    schedule = [dict(base['schedule'][0]), dict(base['schedule'][1]), {'from_s': 9.0, 'to_s': DURATION, 'speed_mps': 0.35}]
    case = {'name': CASE, 'behavior_kind': 'stop_restart', 'route_kind': 'first_curve', 'duration_s': DURATION,
            'schedule': schedule,
            'expected': '진단: 15 s까지 기존 정지·재개, 그 뒤 앞차를 따라 첫 U자 코너 진입·통과 또는 교착 위치/사유 확인'}
    cfg['cases'] = [case]
    rules = dict(stop['acceptance']['stop_restart'])
    for key in ('required_exit_s_m', 'maximum_actor_s_m', 'minimum_heading_change_deg'):
        rules[key] = parent['acceptance']['first_curve'][key]
    rules['basis'] = ('정지·재개 수치는 g1_traffic_v1 stop_restart, 코너 출구·방향 변화·앞차 끝 제한은 extensions_v1 first_curve와 같다. '
                      '내장 final_gap/progress_after_restart는 40 s 창 값이며 등록 정지·재개 판정은 사후 15 s 창이다.')
    cfg['acceptance'][CASE] = rules
    cfg['matrix'] = {'m3': {'cases': [CASE], 'seeds': [0]}, 'pilot': '1회 진단. 재시도 없음.'}
    assert stop['actor'] == parent['actor'] or stop['actor']['start_center_gap_m'] == parent['actor']['start_center_gap_m']
    overlay = json.loads((ROOT/OVERLAY_PARENT).read_text(encoding='utf-8'))
    new = copy.deepcopy(overlay)
    new['registered_at'] = cfg['registered_at']
    new['base_case'] = CASE
    new['scope'] = ('run19/run27과 같은 잔디 쪽 옆차가 15 s까지 같은 속도로 이동하고 경로 종점에 그대로 정지한다. '
                    '순간 소멸·새 초기 배치·경로 연장 없음. 종점 정지는 run27에 없던 새 자극이다.')
    new['parent_overlay'] = {'path': OVERLAY_PARENT, 'sha256': sha(OVERLAY_PARENT)}
    actor = new['additional_actors'][0]
    assert actor['schedule'] == [{'from_s': 0.0, 'to_s': 15.0, 'speed_mps': 0.35}]
    actor['schedule'] = [{'from_s': 0.0, 'to_s': 15.0, 'speed_mps': 0.35}, {'from_s': 15.0, 'to_s': DURATION, 'speed_mps': 0.0}]
    for out, data in ((CONFIG, cfg), (OVERLAY, new)):
        (ROOT/out).write_bytes((json.dumps(data, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
        print('wrote', out, sha(out))


if __name__ == '__main__':
    main()
