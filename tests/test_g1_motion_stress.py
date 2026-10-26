"""모터축 시험 감사·무주입/중복 발행 거절과 실행 전 CLI 경계."""
import hashlib
import json
import math

import pytest

from arena_vehicle_interface.motion_impairment import MotionFaultConfig, StreamFault
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
from arena_vehicle_interface.motor_count_impairment import MODEL
from motion_fault_audit import audit
from g1_motion_stress import load_motion_profile, require_motion_evidence, motor_count_resolution
import validate_g1_traffic as harness


def log_rows(mode='drive_motor_shaft',count=1):
    cfg=MotionFaultConfig(imu=StreamFault(bias=.01),wheels=StreamFault(scale=.95))
    rows=[dict(event='config',config=cfg.as_dict(),feedback_mode=mode)]
    for kind,n in (('imu',1),('wheels',count)):
        for i in range(30):
            t=i*.01
            rows.append(dict(event='publish',stream=kind,source_stamp_s=t,received_s=t,released_s=t,
                             due_s=t,delay_s=0.,claimed_stamp_s=t,fault_active=True,
                             before=[1.]*n,after=[1.01 if kind=='imu' else .95]*n))
    rows.append(dict(event='close'))
    return cfg.as_dict(),rows


def write_log(tmp_path,rows):
    path=tmp_path/'motion_injection.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in rows))
    return path


def test_single_motor_audit_and_legacy_record_compatibility(tmp_path):
    cfg,rows=log_rows()
    path=write_log(tmp_path,rows)
    assert audit(path,cfg,feedback_mode='drive_motor_shaft')['verified']
    assert audit(path,cfg)['feedback_mode']=='drive_motor_shaft'
    assert not audit(path,cfg,feedback_mode='rear_wheel_pair_legacy')['verified']
    cfg,rows=log_rows('rear_wheel_pair_legacy',2)
    del rows[0]['feedback_mode']  # 9/22 기존 기록 호환만 유지
    path=write_log(tmp_path,rows)
    assert audit(path,cfg)['verified']
    assert not audit(path,cfg,feedback_mode='drive_motor_shaft')['verified']


@pytest.mark.parametrize('fault',['two_channels','wrong_value','wrong_stamp','wrong_delay','missing_stream','no_close','bad_mode'])
def test_injection_audit_rejects_invalid_evidence(tmp_path,fault):
    cfg,rows=log_rows(count=2 if fault=='two_channels' else 1)
    if fault=='wrong_value': rows[1]['after']=[1.]
    if fault=='wrong_stamp': rows[1]['claimed_stamp_s']=.001
    if fault=='wrong_delay': rows[1]['delay_s']=.01
    if fault=='missing_stream': rows=[r for r in rows if r.get('stream')!='wheels']
    if fault=='no_close': rows.pop()
    if fault=='bad_mode': rows[0]['feedback_mode']='unknown'
    assert not audit(write_log(tmp_path,rows),cfg,feedback_mode='drive_motor_shaft')['verified']


def ready_report():
    return dict(integrity={},harness_valid=True,run_valid=True,passed=True,
                motion_test_profile={'counts_per_motor_revolution':28},
                autonomy_subscriptions=['/imu/data','/drive_motor/encoder'],
                safety_subscriptions=['/imu/data','/drive_motor/encoder'],
                motion_relay_publishers={k:['motion_fault_relay'] for k in ('/imu/data','/drive_motor/encoder')})


@pytest.mark.parametrize('problem',[None,'publisher','consumer','profile','previous_failure','missing_log'])
def test_traffic_verdict_cannot_hide_bad_injection_or_prior_failure(tmp_path,problem):
    cfg,rows=count_log_rows(); write_log(tmp_path,rows)
    profile=tmp_path/'motion_test_profile.json'; profile.write_text(json.dumps(cfg))
    digest=hashlib.sha256(profile.read_bytes()).hexdigest()
    report=ready_report()
    if problem=='publisher': report['motion_relay_publishers']['/imu/data'].append('raw_bridge')
    if problem=='consumer': report['safety_subscriptions'].remove('/drive_motor/encoder')
    if problem=='profile': profile.write_text('{}')
    if problem=='previous_failure': report['passed']=False
    if problem=='missing_log': (tmp_path/'motion_injection.jsonl').unlink()
    require_motion_evidence(report,tmp_path,cfg,digest)
    assert report['passed']==(problem is None)
    if problem not in (None,'previous_failure'):
        assert not report['run_valid'] and not report['harness_valid']


def test_profile_default_and_outside_repo_rejection(tmp_path):
    assert load_motion_profile(None,tmp_path)==(None,None)
    root=tmp_path/'repo'; root.mkdir()
    other=tmp_path/'profile.json'; other.write_text('{}')
    with pytest.raises(ValueError): load_motion_profile(other,root)


@pytest.mark.parametrize('extra',[
    ['--mode','r0'],['--mode','g1'],
    ['--mode','g1','--record-follow-decisions','--lidar-test-profile','config/tests/lidar_uniform30_delay50_jitter20.json'],
    ['--mode','g1','--record-follow-decisions','--actor-overlay','overlay.json'],
    ['--mode','g1','--record-follow-decisions','--race-operation-test']])
def test_incompatible_motion_trial_rejected_before_launch(monkeypatch,tmp_path,extra):
    monkeypatch.setattr('sys.argv',['trial','--case','stop_restart','--seed','0','--output',str(tmp_path),
                                  '--motion-test-profile','config/tests/motion_combined.json',*extra])
    monkeypatch.setattr(harness,'load_motion_profile',lambda *_:pytest.fail('must reject before loading'))
    with pytest.raises(SystemExit) as error: harness.main()
    assert error.value.code==2


@pytest.mark.parametrize('mode',['g1','m3'])
def test_explicit_recorded_trial_reaches_profile_before_launch(monkeypatch,tmp_path,mode):
    monkeypatch.setattr('sys.argv',['trial','--case','curve_stop_restart','--seed','0','--mode',mode,
                                  '--record-follow-decisions','--occupancy-timing','per_ray_candidate',
                                  '--motion-test-profile','config/tests/motion_combined.json','--output',str(tmp_path)])
    def reached(path,repo):
        assert path.name=='motion_combined.json'
        raise RuntimeError('profile_without_launch')
    monkeypatch.setattr(harness,'load_motion_profile',reached)
    with pytest.raises(RuntimeError,match='profile_without_launch'): harness.main()


def test_source_snapshot_includes_profile_relay_and_auditor():
    paths=harness.source_paths(harness.CONFIG,motion_profile='config/tests/motion_combined.json')
    assert {'config/tests/motion_combined.json','scripts/g1_motion_stress.py','scripts/motion_fault_audit.py',
            'src/arena_vehicle_interface/arena_vehicle_interface/motion_fault_relay.py'} <= set(paths)


def test_motion_trial_reads_actual_vehicle_configuration():
    assert motor_count_resolution(harness.REPO)==28


def count_log_rows():
    cfg,rows=log_rows()
    spec=MotorEncoderSpec(28,15.,.0325)
    rows[0].update(motor_count_model=MODEL,counts_per_motor_revolution=28)
    previous=0
    for i,r in enumerate(row for row in rows if row.get('stream')=='wheels'):
        raw=i*10; count=round(raw*.95)
        r.update(before_position=[spec.motor_angle_from_counts(raw)],after_position=[spec.motor_angle_from_counts(count)],
                 before=[None if i==0 else 10*spec.motor_rad_per_count/.01],
                 after=[None if i==0 else (count-previous)*spec.motor_rad_per_count/.01],
                 before_velocity_kind='finite' if i else 'nan', after_velocity_kind='finite' if i else 'nan')
        previous=count
    return cfg,rows


@pytest.mark.parametrize('mutation',[None,'off_grid','wrong_count','wrong_speed','later_nan','first_inf','wrong_resolution','missing_model'])
def test_integer_count_audit_rejects_corruption(tmp_path,mutation):
    cfg,rows=count_log_rows()
    wheels=[r for r in rows if r.get('stream')=='wheels']
    if mutation=='off_grid':wheels[1]['after_position'][0]+=.001
    if mutation=='wrong_count':wheels[1]['after_position'][0]+=math.tau/28
    if mutation=='wrong_speed':wheels[1]['after'][0]*=.95
    if mutation=='later_nan':wheels[1].update(after=[None],after_velocity_kind='nan')
    if mutation=='first_inf':wheels[0]['before_velocity_kind']='infinite'
    if mutation=='wrong_resolution':rows[0]['counts_per_motor_revolution']=27
    if mutation=='missing_model':del rows[0]['motor_count_model']
    result=audit(write_log(tmp_path,rows),cfg,feedback_mode='drive_motor_shaft',counts_per_motor_revolution=28)
    assert result['verified']==(mutation is None)
