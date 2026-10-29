"""연속 랩·시작선 왕복·누락 반례. 실제 차량 주행 검사가 아니다."""
from copy import deepcopy
import math

import numpy as np
import pytest
from shapely.geometry import box

import course_candidate as candidate
import g1_course_eval as original
from test_g1_course_eval import circle, poses


def configuration(laps):
    line = circle()
    duration = laps*line.lap+.5
    case = dict(name='registered_laps', route_kind='full_course', duration_s=duration,
                schedule=[dict(from_s=0., to_s=duration, speed_mps=1.)])
    config = dict(actor=dict(length_m=.2, width_m=.15), acceptance={case['name']:dict(
        minimum_laps=laps, maximum_actor_laps=laps+.2,
        surface_policy='two_wheel_centers', wheelbase_m=.135, track_m=.135)})
    return line, config, case


def test_existing_one_lap_preflight_result_is_unchanged():
    line, config, case = configuration(1)
    args = (config, line, case, line.lap-.2, box(-3,-3,3,3), [])
    assert candidate.check_full_course(*args) == original.check_full_course(*args)


def test_explicit_five_lap_checks_whole_actor_route():
    line, config, case = configuration(5)
    result = candidate.check_full_course(config,line,case,0.,box(-3,-3,3,3),[])
    assert result['minimum_laps'] == 5
    assert result['actor_end_s_m'] > 5*line.lap
    assert result['actor_road_wall_samples'] == math.ceil(case['duration_s']/.01)+1
    with pytest.raises(ValueError):
        original.check_full_course(config,line,case,0.,box(-3,-3,3,3),[])


@pytest.mark.parametrize('laps', [True, 0, 2, 4, 5.0, 6])
def test_unregistered_lap_count_is_rejected(laps):
    line, config, case = configuration(5)
    config['acceptance'][case['name']]['minimum_laps'] = laps
    with pytest.raises(ValueError):
        candidate.check_full_course(config,line,case,0.,box(-3,-3,3,3),[])


@pytest.mark.parametrize('fault',['short','unbounded','surface','wall','no_road'])
def test_long_run_preserves_route_and_observation_conditions(fault):
    line, config, case = configuration(5)
    road, walls = box(-3,-3,3,3), []
    rules = config['acceptance'][case['name']]
    if fault == 'short':
        case['duration_s'] = 4*line.lap
        case['schedule'][0]['to_s'] = case['duration_s']
    if fault == 'unbounded': rules['maximum_actor_laps'] = 7.
    if fault == 'surface': rules['surface_policy'] = 'ignore'
    if fault == 'wall': walls = [box(-3,-3,3,3)]
    if fault == 'no_road': road = None
    with pytest.raises(ValueError):
        candidate.check_full_course(config,line,case,0.,road,walls)


def test_five_lap_progress_is_continuous_and_missing_actor_is_not_filled():
    line = circle()
    own = poses(line,line.lap-.8,5*line.lap+.1)
    lead = poses(line,line.lap-.2,5*line.lap+.1)
    result = original.full_course_progress(line,own,lead,line.lap-.2,.2)
    assert result['valid'] and result['ego_stays_behind']
    assert result['progress_m'] >= 5*line.lap
    assert result['ego_end_s_m'] > 5*line.lap
    missing = lead[:1000]+lead[1010:]
    assert not original.full_course_progress(line,own,missing,line.lap-.2,.2)['valid']


def test_repeated_start_line_crossings_do_not_count_as_laps():
    line = circle()
    offsets = np.concatenate([np.linspace(-.1,.1,30),np.linspace(.1,-.1,30)]*6)
    own = [(i*.02,*line.pose(line.lap+s)) for i,s in enumerate(offsets)]
    lead = [(i*.02,*line.pose(line.lap+s+.6)) for i,s in enumerate(offsets)]
    result = original.full_course_progress(line,own,lead,line.lap+.6,.2)
    assert result['valid'] and abs(result['progress_m']) < .02
    reverse = poses(line,line.lap-.8,2.)[::-1]
    assert not original.continuous_course_positions(line,reverse,line.lap-.8)[0]
