"""실제 턱 방문과 기존 교통 실패를 구별하는 사후 요약 검사."""
import importlib.util
import json
import math
from pathlib import Path

import pytest

BASE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('bump_traffic_analysis_test', BASE/'analyze.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def make_input(target, *, lateral=0., passed=True, reverse=False, empty=False):
    scene = analysis.ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture/scene.json'
    bump = json.loads(scene.read_text(encoding='utf-8'))['speed_bumps']['bumps'][0]
    yaw = bump['yaw_rad']
    c, s = math.cos(yaw), math.sin(yaw)
    poses, trace = [], []
    for i in range(201):
        along = (-.5+i*.005)*(-1 if reverse else 1)
        t = i*.01
        poses.append(dict(time=t, position=dict(x=bump['x']+c*along-s*lateral,
            y=bump['y']+s*along+c*lateral, z=.065),
            orientation=dict(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))
        trace.append(dict(t=t, ground_speed_mps=.5, safe_speed_mps=.5, safety_reason='clear'))
    if empty:
        poses, trace = [], []
    (target/'bump_pose_trace.json').write_text(json.dumps(poses), encoding='utf-8')
    (target/'trajectory.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in trace), encoding='utf-8')
    report = dict(passed=passed, evaluation=dict(gates=dict(full_lap=passed)))
    (target/'report.json').write_text(json.dumps(report), encoding='utf-8')
    (target/'trial_overlay.json').write_text(json.dumps(dict(source_unchanged=True,
        runtime_contract=dict(passed=True))), encoding='utf-8')


def test_forward_pass_uses_all_four_wheels_and_preserves_raw(tmp_path):
    make_input(tmp_path)
    result = analysis.analyze(tmp_path)
    assert result['limited_passed'] and result['input_unchanged']
    assert len(result['visits']) == 1
    visit = result['visits'][0]
    assert all(w['passed'] for w in visit['crossing']['wheels'].values())
    assert visit['sampled_body_speed_max_mps'] == .5
    assert visit['safe_zero_samples'] == 0


@pytest.mark.parametrize('case', ['far_road', 'reverse', 'empty', 'traffic_failed'])
def test_missing_crossing_or_failed_traffic_is_not_integrated_success(tmp_path, case):
    make_input(tmp_path, lateral=2. if case=='far_road' else 0.,
        passed=case!='traffic_failed', reverse=case=='reverse', empty=case=='empty')
    result = analysis.analyze(tmp_path)
    assert not result['limited_passed'] and result['input_unchanged']
    if case == 'traffic_failed':
        assert result['checks']['geometry_visit']
        assert result['original_failed_gates'] == ['full_lap']
    else:
        assert not result['checks']['geometry_visit']
