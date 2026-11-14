"""host/compiled-MCU profile 정합 및 미교정 부호 중복 적용 금지."""
import hashlib
import json
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'tests')]
from test_uno_q_host import make_owner, ticks, arm
from uno_q_profile import matching_mcu_profile, matching_motor_feedback, ProfileGatedMotorFeedback


def profiles():
    board = dict(assembly_id='synthetic', encoder_direction=-1, motor_direction_confirmed=True,
                 forward_dir_high=False, maximum_duty=.15, kp=.1, ki=.01, maximum_speed_mps=.5,
                 feedback_period_us=20000, servo=dict(command_limit=.2))
    mcu = {key: value for key, value in board.items() if key != 'servo'}
    mcu.update(schema='uno_q_profile_v1', mode='installed', maximum_steering_rad=.2)
    return board, mcu


@pytest.mark.parametrize('key,value', [('encoder_direction', 1), ('forward_dir_high', True),
    ('kp', .2), ('ki', .02), ('maximum_duty', .2), ('maximum_speed_mps', .6),
    ('maximum_steering_rad', .25), ('feedback_period_us', 10000), ('assembly_id', 'other')])
def test_profile_mismatch_rejected_before_any_device(tmp_path, key, value):
    board, mcu = profiles();mcu[key] = value
    path = tmp_path/'mcu.json';path.write_text(json.dumps(mcu))
    with pytest.raises(ValueError, match='mismatch'):matching_mcu_profile(path, board)


def test_profile_fingerprint_matches_firmware_canonical_json(tmp_path):
    board, mcu = profiles();path = tmp_path/'mcu.json';path.write_text(json.dumps(mcu, indent=4))
    expected = hashlib.sha256(json.dumps(mcu, sort_keys=True, separators=(',', ':'),
                                        ensure_ascii=True, allow_nan=False).encode('ascii')).hexdigest()
    assert matching_mcu_profile(path, board) == expected


def bad_hash_observation(mcu):
    mcu.row(dict(type='uno_q', v=1, boot=mcu.boot, generation=mcu.generation, accepted_seq=mcu.seq,
        state=mcu.state, reason='fixture', bench_state='disarmed', bench_seq=-1,
        bench_reason='disarmed', profile_sha256='b'*64, mode='installed'))


def test_fresh_wrong_compiled_hash_blocks_arm_before_torque_and_active_change_revokes():
    owner, t, m, u, _ = make_owner();ticks(owner, t, 7)
    bad_hash_observation(m);ticks(owner, t)
    assert owner.request_control('ARM')['reason'] == 'fresh_compiled_MCU_profile_match_required'
    assert not any(w[4] == 3 for w in u.sent)
    owner, t, m, u, _ = make_owner();arm(owner, t)
    bad_hash_observation(m);ticks(owner, t)
    assert owner.terminating and owner.authorization is None and 'profile' in owner.exit_reason


def test_missing_or_stale_compiled_hash_blocks_arm():
    owner, t, m, u, _ = make_owner();ticks(owner, t, 7)
    owner.expected_profile_hash = None
    assert owner.request_control('ARM')['reason'] == 'fresh_compiled_MCU_profile_match_required'
    owner.expected_profile_hash = 'a'*64
    owner.profile_observation['receipt_ns'] = t.now-100_000_000
    assert owner.request_control('ARM')['reason'] == 'fresh_compiled_MCU_profile_match_required'


def test_old_startup_observation_does_not_refresh_profile_or_abort_diagnostics():
    owner, t, m, u, _ = make_owner(False)
    t.step(40_000_000);ticks(owner, t)
    assert not owner.terminating and owner.profile_observation is None
    assert owner.counts['stale_uno_observation'] == 1


def test_signed_mcu_feedback_must_not_be_inverted_twice():
    data = dict(counts_per_motor_revolution=28, gear_ratio=2.5*81/17, wheel_radius_m=.0325, encoder_sign=1)
    matching_motor_feedback(dict(declared=data))
    for key, value in (('encoder_sign', -1), ('counts_per_motor_revolution', 112),
                       ('gear_ratio', 5.), ('wheel_radius_m', .03)):
        with pytest.raises(ValueError):matching_motor_feedback(dict(declared=data|{key: value}))


def test_unconfigured_feedback_cannot_be_velocity_and_identity_gap_reanchors():
    from test_motor_feedback_runtime import converter, sample
    current = {'matches': False}
    gated = ProfileGatedMotorFeedback(converter(), lambda _: current['matches'])
    assert gated.convert(sample()) is None
    assert gated.convert(sample(2, 5, 120000)) is None
    current['matches'] = True
    assert gated.convert(sample(3, 7, 140000)) is None
    assert gated.convert(sample(4, 8, 160000)).velocity_rad_s > 0
    current['matches'] = False
    assert gated.convert(sample(5, 100, 180000)) is None
    current['matches'] = True
    assert gated.convert(sample(6, 101, 200000)) is None
