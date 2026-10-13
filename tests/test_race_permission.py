import math

import pytest

from arena_autonomy.race_permission import RacePermission


def start(gate):
    assert gate.observe_start(race_epoch=gate.race_epoch, visual_confirmed=True,
                              capture_s=1., now_s=1.1, max_age_s=1.)


def decide(gate, **overrides):
    return gate.decide(**dict(dict(sensors_ready=True, actuator_ready=True), **overrides))


def test_arm_alone_does_not_start_and_temporary_hold_recovers_without_new_signal():
    gate = RacePermission()
    assert decide(gate).reason == 'wait_visual_start'
    start(gate)
    assert decide(gate).allowed
    assert not decide(gate, sensors_ready=False).allowed
    assert not decide(gate, temporary_hold=True).allowed
    assert decide(gate).allowed


def test_stop_needs_explicit_matching_ack_and_separate_mcu_readiness():
    gate = RacePermission()
    start(gate)
    token = gate.stop()
    assert not decide(gate).allowed
    assert not gate.observe_start(race_epoch=0, visual_confirmed=True, capture_s=2., now_s=2., max_age_s=1.)
    assert gate.acknowledge(race_epoch=0, intervention=token, hazards_cleared=True, operator_reviewed=True)
    assert not decide(gate, actuator_ready=False).allowed
    assert decide(gate).allowed


def test_stale_ack_cannot_clear_later_stop():
    gate = RacePermission()
    token = gate.stop()
    gate.stop()
    assert not gate.acknowledge(race_epoch=0, intervention=token, hazards_cleared=True, operator_reviewed=True)
    assert not decide(gate).allowed


@pytest.mark.parametrize('field,value', [('hazards_cleared', False), ('operator_reviewed', False),
                                        ('race_epoch', 10), ('intervention', True)])
def test_incomplete_ack_does_not_clear(field, value):
    gate = RacePermission()
    token = gate.fault('vision')
    args = dict(race_epoch=0, intervention=token, hazards_cleared=True, operator_reviewed=True)
    args[field] = value
    assert not gate.acknowledge(**args)
    assert decide(gate).reason == 'fault:vision'


def test_new_race_revokes_visual_permit_but_does_not_clear_stop():
    gate = RacePermission()
    start(gate)
    token = gate.stop()
    epoch = gate.new_race()
    assert not gate.started and not decide(gate).allowed
    assert not gate.acknowledge(race_epoch=0, intervention=token, hazards_cleared=True, operator_reviewed=True)
    assert gate.acknowledge(race_epoch=epoch, intervention=token, hazards_cleared=True, operator_reviewed=True)
    assert not decide(gate).allowed


@pytest.mark.parametrize('override', [{'capture_s': math.nan}, {'capture_s': 2.}, {'capture_s': 0.},
                                    {'max_age_s': 0.}, {'visual_confirmed': False}, {'race_epoch': 1},
                                    {'now_s': math.inf}, {'race_epoch': True}])
def test_invalid_start_evidence_rejected(override):
    gate = RacePermission()
    args = dict(race_epoch=0, visual_confirmed=True, capture_s=1., now_s=1.1, max_age_s=1.)
    args.update(override)
    assert not gate.observe_start(**args)
    assert not decide(gate).allowed


def test_duplicate_evidence_is_not_new_authority_and_restart_is_fail_closed():
    gate = RacePermission()
    start(gate)
    assert not gate.observe_start(race_epoch=0, visual_confirmed=True, capture_s=1., now_s=1.2, max_age_s=1.)
    assert not decide(RacePermission()).allowed
    gate.clock_reset()
    assert not decide(gate).allowed and not gate.started


def test_ambiguous_boolean_inputs_fail_closed():
    gate = RacePermission()
    start(gate)
    assert not decide(gate, actuator_ready=1).allowed
    assert not decide(gate, temporary_hold=None).allowed


def test_stop_reason_cannot_be_used_as_permission():
    gate = RacePermission()
    start(gate)
    gate.stop('race_permitted')
    assert not decide(gate).allowed
