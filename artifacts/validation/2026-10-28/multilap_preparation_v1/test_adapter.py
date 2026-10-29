"""5랩 설정 전달과 평가 전용 연결. ROS 주행을 시작하지 않는다."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest

BASE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('five_lap_adapter_test', BASE/'run.py')
run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(run)


def test_configuration_keeps_existing_drive_rules_and_extends_only_the_end():
    current, overlay, case = run.registered_config()
    old = json.loads((BASE.parent/'bump_traffic_v1/traffic_config.json').read_text(encoding='utf-8'))
    assert current['launch'] == old['launch']
    for name in ('actor','seeds','vehicle_and_sensor_unchanged'):
        assert current[name] == old[name]
    expected = deepcopy(old['acceptance'])
    expected[case['name']].update(minimum_laps=5, maximum_actor_laps=5.2)
    assert current['acceptance'] == expected
    expected_case = deepcopy(old['cases'][0])
    expected_case['schedule'][-1]['to_s'] = 685.
    assert case['schedule'] == expected_case['schedule']
    assert overlay['additional_actors'][0]['schedule'][0] == dict(from_s=0.,to_s=15.,speed_mps=.35)
    assert overlay['additional_actors'][0]['schedule'][1] == dict(from_s=15.,to_s=685.,speed_mps=0.)


def test_launch_uses_same_bump_and_sensor_candidate_with_fresh_output():
    output = BASE/'not_executed'
    text, settings, arguments = run.parent.launch_source(output)
    assert arguments['speed_bump'] == 'true'
    assert arguments['lidar_rate_hz'] == '10.0'
    assert arguments['speed_profile'] == 'g1_compare_1mps'
    assert settings['g1_trace_path'] == str(output/'follow_decisions.jsonl')
    assert settings['start_signal_mode'] == 'official_4red_local_candidate'
    assert 'course_candidate' not in text  # evaluator hook is not sent to control nodes
    assert run.parent.CONFIG == BASE/'traffic_config.json'
    assert run.parent.OVERLAY == BASE/'actor_overlay.json'


@pytest.mark.parametrize('fault',[None,'duration','laps','extent','config'])
def test_runtime_contract_detects_requested_versus_applied_scope(monkeypatch,fault):
    monkeypatch.setattr(run,'original_verify',lambda *args:dict(passed=True))
    metadata=dict(route=dict(minimum_laps=5),route_hook=dict(evaluation_only=True))
    report=dict(config_sha256=run.parent.sha(run.parent.CONFIG),
        case_window=dict(green_sim_s=7.,end_sim_s=692.02,duration_s=685.),
        route_check=dict(minimum_laps=5, maximum_actor_laps=5.2))
    if fault == 'duration': report['case_window']['duration_s']=155.
    if fault == 'laps': report['route_check']['minimum_laps']=1
    if fault == 'extent': report['route_check']['maximum_actor_laps']=1.2
    if fault == 'config': report['config_sha256']='wrong'
    result=run.verify_runtime(BASE/'not_executed',metadata,report)
    assert result['passed'] is (fault is None)


@pytest.mark.parametrize('raises',[False,True])
def test_route_hook_is_process_local_and_restored_even_on_failure(monkeypatch,raises):
    import g1_course_eval
    import course_candidate
    original=g1_course_eval.check_full_course
    def fake(output):
        assert g1_course_eval.check_full_course is course_candidate.check_full_course
        if raises: raise RuntimeError('synthetic failure')
        return 17
    monkeypatch.setattr(run.parent,'run',fake)
    if raises:
        with pytest.raises(RuntimeError,match='synthetic'): run.run(BASE/'not_executed')
    else:
        assert run.run(BASE/'not_executed') == 17
    assert g1_course_eval.check_full_course is original
