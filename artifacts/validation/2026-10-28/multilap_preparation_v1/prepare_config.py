"""5랩 비교 설정을 별도 파일에 보존한다. 시뮬레이션 실행이나 기본값 변경 없음."""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]


def load(relative):
    path = ROOT/relative
    return json.loads(path.read_text(encoding='utf-8')), dict(
        path=relative, sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def write(name, value):
    with (BASE/name).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    config, parent = load('artifacts/validation/2026-10-28/bump_traffic_v1/traffic_config.json')
    overlay, overlay_parent = load('config/tests/e3_stop_restart_full_course_v1.json')
    duration = 685.
    case = config['cases'][0]
    assert case['name'] == 'stop_restart_full_course' and case['duration_s'] == 155.
    case['duration_s'] = duration
    case['schedule'][-1]['to_s'] = duration
    case['expected'] = '공식 신호·방지턱·동일 각본 교통의 순진행5랩과 명시STOP'
    rules = config['acceptance'][case['name']]
    rules['minimum_laps'] = 5
    rules['maximum_actor_laps'] = 5.2
    config['registered_at'] = '2026-10-28 before five-lap trial, preparation only'
    config['scope'] = '명시685초·5랩 통합 후보. 평가기 종료이며 차량의 자체 랩 인식/실물 검증 아님.'
    config['straight_section_check'] = 'full_course 명시5랩 검증을 통해 등록 앞차 경로를1cm마다 도로/벽 대조.'
    config['matrix']['pilot'] = '685s 1회. 실패 원인 변화 없이 자동 재시도하지 않음.'
    config['timeouts']['wall_timeout_s'] = 10800
    config['parent_config'] = parent
    config['five_lap_trial'] = dict(enabled=True, duration_s=duration,
        required_ego_laps=5, baseline_lap_length_m=46.6329,
        configured_actor_travel_m=238., evaluation_only=True,
        control_settings_changed=False,
        scope='one scripted lead and a stopped grass-side actor, not five autonomous cars')
    overlay['additional_actors'][0]['schedule'][-1]['to_s'] = duration
    overlay['registered_at'] = config['registered_at']
    overlay['scope'] = '같은 잔디 쪽 표적을685초 동안 보존하여 다랩 재조우를 관찰한다.'
    overlay['parent_overlay'] = overlay_parent
    write('traffic_config.json', config)
    write('actor_overlay.json', overlay)
