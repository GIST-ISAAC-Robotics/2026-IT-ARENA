import copy
import math
import pytest

from probe_follow_cadence import Cadence, measured_costs


def test_busy_discards_intermediate_ticks_and_admits_first_available():
    c = Cadence()
    assert c.offer(1., .075, .02) == (.02, 1.075)
    assert [c.offer(t, .075, .02) for t in (1.02, 1.04, 1.06)] == [None]*3
    assert c.offer(1.08, .07, .02) == pytest.approx((.02, 1.15))
    assert c.previous_start == 1.08


def test_elapsed_dt_preserves_overrun_instead_of_clamping():
    c = Cadence(dt_mode='elapsed')
    c.offer(1., .25, .02)
    assert c.offer(1.2, .01, .02) is None
    dt, done = c.offer(1.26, .02, .02)
    assert dt == pytest.approx(.26)
    assert done == 1.28


def test_reset_removes_busy_and_previous_dt_and_accepts_new_clock():
    c = Cadence(dt_mode='elapsed')
    c.offer(10., 1., .02)
    c.reset()
    assert c.offer(.1, .05, .02) == pytest.approx((.02, .15))


def test_zero_cost_or_all_mode_preserves_all_ticks():
    for c, cost in ((Cadence(), 0.), (Cadence('all'), 1.)):
        for t in (1., 1.02, 1.04):
            assert c.offer(t, cost, .02) == (.02, t)


@pytest.mark.parametrize('values', [(math.nan, .1, .02), (1., math.inf, .02),
    (1., -.1, .02), (1., .1, 0.), (-1., .1, .02), (True, .1, .02)])
def test_bad_time_is_not_silently_repaired(values):
    with pytest.raises(ValueError):
        Cadence().offer(*values)


def test_reversed_clock_requires_explicit_boundary():
    c = Cadence()
    c.offer(2., .1, .02)
    with pytest.raises(ValueError):
        c.offer(1., .1, .02)


def cost_report():
    return dict(input_unchanged=True, source_unchanged=True, calls=1,
        per_call_stages=[dict(seq=1, stages={name:dict(calls=1, wall_ms=value)
            for name,value in [('restore_input', 100.), ('motion_scene', 6.), ('follow_command', 60.)]})])


def test_cost_excludes_offline_restore_but_keeps_motion():
    assert measured_costs(cost_report()) == {1: .066}


@pytest.mark.parametrize('failure', ['duplicate', 'missing', 'source', 'stage_count', 'negative'])
def test_invalid_cost_evidence_rejected(failure):
    r = cost_report()
    if failure == 'duplicate':
        r['per_call_stages'].append(copy.deepcopy(r['per_call_stages'][0])); r['calls'] = 2
    elif failure == 'missing':
        r['calls'] = 2
    elif failure == 'source':
        r['source_unchanged'] = False
    elif failure == 'stage_count':
        r['per_call_stages'][0]['stages']['motion_scene']['calls'] = 0
    else:
        r['per_call_stages'][0]['stages']['follow_command']['wall_ms'] = -1.
    with pytest.raises(ValueError):
        measured_costs(r)
