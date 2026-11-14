import json
from pathlib import Path
import tempfile
import zipfile

import pytest
import prepare_update as subject
from test_runtime_installed_limits import synthetic_bundle


@pytest.fixture
def root():
    with tempfile.TemporaryDirectory(prefix='r533-synthetic-') as directory:
        yield Path(directory)


@pytest.fixture
def source_fixture(monkeypatch):
    tool=subject.collector()
    files={name:('SYNTHETIC SOURCE '+name+'\n').encode() for name in tool.REQUIRED}
    monkeypatch.setattr(tool,'collect',lambda repo:dict(files))
    monkeypatch.setattr(subject,'collector',lambda:tool)
    monkeypatch.setattr(subject,'baseline',lambda:dict(files={'scripts/old.py':'original-hash'}))
    return files,tool


def physical_inputs(root):
    installed=synthetic_bundle(root)
    base={name:{'ros__parameters':dict(max_steering_angle_rad=.4,max_speed_mps=.1,
             g1_g2_moving_pass_trial=False,g1_g2_moving_pass_profile='slow_v1')}
          for name in subject.policy.limits.NODES}
    bp=root/'base.yaml';bp.write_text(subject.policy.yaml.safe_dump(base),encoding='utf-8')
    inputs=subject.policy.required_inputs()
    inputs.update(assembly_id='SYNTHETIC-ONLY',runtime_port='NO_DEVICE',clock_policy='NO_CLOCK_FILE',motor_feedback_config='NO_FEEDBACK_FILE')
    inputs['shared_ros_parameters']={key:('synthetic_imu' if key.endswith('frame_id') else .01)
                                     for key in subject.policy.PHYSICAL}
    inputs['shared_ros_parameters']['motion_lidar_frame_id']='c1_unmounted_preview'
    ip=root/'hardware.json';subject.save(ip,inputs)
    return dict(base_params=bp,hardware_inputs=ip,board_profile=installed/'runtime_board.json',
                steering_profile=installed/'steering_zero.json')


def test_existing_collector_and_r500_original_are_preserved():
    assert subject.collector().inventory({})['source_only'] is True
    assert len(subject.baseline()['files'])==721


def test_draft_selects_compact_but_keeps_physical_values_null(root):
    from arena_autonomy.follow_trace import FollowTrace
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.local_follow import FollowConfig
    from arena_autonomy.bypass_handoff import HandoffExecutionConfig
    subject.policy.draft(root/'draft')
    overlay=subject.policy.policy_overlay();subject.policy.check_policy(overlay)
    assert overlay['local_pursuit']['ros__parameters']['g1_trace_detail']=='compact_v1'
    for node in subject.policy.limits.NODES:
        p=overlay[node]['ros__parameters']
        assert p['race_control_mode']=='b3_candidate' and p['g1_guard_observation_policy']=='selected_r181'
        assert p['motion_scan_timing_verified'] is False
    pending=subject.load(root/'draft/required_inputs.json')
    assert all(v is None for v in pending['shared_ros_parameters'].values())
    assert pending['runtime_port'] is None
    trace=FollowTrace(root/'compact.jsonl',MotionConfig(),FollowConfig(),mode='g1_g2_handoff_trial',
        execution_config=HandoffExecutionConfig(),detail=overlay['local_pursuit']['ros__parameters']['g1_trace_detail'])
    trace.close()
    assert json.loads((root/'compact.jsonl').read_text().splitlines()[0])['data']['replayable'] is False


def test_source_only_one_command_reuses_collector_and_rejects_later_drift(root,source_fixture):
    files,tool=source_fixture
    output=root/'candidate'
    result=subject.prepare(output,'SYNTHETIC final selection')
    assert result['ok'] and result['install_pending'] and not result['hardware_selection_prepared']
    assert subject.load(output/'verification_receipt.json')==result
    with zipfile.ZipFile(output/'runtime_handover_sources.zip') as zipped:
        assert 'handover/prepare_hardware_policy.py' in zipped.namelist()
        assert json.loads(zipped.read('handover/required_inputs.json'))['assembly_id'] is None
    assert not (output/'private').exists()
    files['scripts/new.py']=b'SYNTHETIC changed source'
    assert subject.validate(output)['ok']
    with pytest.raises(ValueError,match='differs from current'):subject.validate(output,against_current=True)
    with pytest.raises(FileExistsError):subject.prepare(output,'do not overwrite')


def test_real_input_path_uses_explicit_mounted_profile_and_private_outputs(root,source_fixture):
    inputs=physical_inputs(root)
    originals={name:path.read_bytes() for name,path in inputs.items()}
    output=root/'candidate'
    result=subject.prepare(output,'SYNTHETIC explicit mounted inputs',**inputs)
    assert result['hardware_selection_prepared'] and not result['installed']
    installed=output/'private/installed_profile'
    assert (installed/'steering_zero.json').read_bytes()==originals['steering_profile']
    profile=subject.policy.limits.profile_values((output/'private/runtime_selection/selected/runtime_profile.yaml').read_bytes())
    assert profile['local_pursuit']['ros__parameters']['g1_trace_detail']=='compact_v1'
    for node in subject.policy.limits.NODES:
        assert profile[node]['ros__parameters']['max_steering_angle_rad']<.2
        assert profile[node]['ros__parameters']['max_speed_mps']==.1
    assert {name:path.read_bytes() for name,path in inputs.items()}==originals
    with zipfile.ZipFile(output/'runtime_handover_sources.zip') as zipped:
        assert all('private/' not in name for name in zipped.namelist())
        assert all(b'SYNTHETIC-ONLY' not in zipped.read(name) for name in zipped.namelist())


def test_neutral_only_and_partial_inputs_cannot_masquerade_as_mounted(root,source_fixture):
    inputs=physical_inputs(root)
    with pytest.raises(ValueError,match='supplied together'):
        subject.prepare(root/'partial','synthetic',board_profile=inputs['board_profile'])
    assert not (root/'partial').exists()
    from arena_vehicle_interface.steering_calibration import record_zero
    zero=root/'straight_only.json'
    subject.save(zero,record_zero(2000,assembly_id='SYNTHETIC-ONLY',straight_confirmed=True,evidence_note='SYNTHETIC ONLY'))
    inputs['steering_profile']=zero
    with pytest.raises(ValueError):subject.prepare(root/'neutral_only','synthetic',**inputs)
    assert not (root/'neutral_only/runtime_handover_sources.zip').exists()


def test_zip_tampering_and_change_during_collection_cannot_validate(root,source_fixture,monkeypatch):
    files,tool=source_fixture
    output=root/'candidate';subject.prepare(output,'synthetic')
    archive=output/'runtime_handover_sources.zip';archive.write_bytes(archive.read_bytes()+b'tamper')
    with pytest.raises(ValueError,match='ZIP hash'):subject.validate(output)
    count=0
    def changing(repo):
        nonlocal count
        count+=1
        return dict(files,**({'scripts/changed.py':b'changed'} if count>1 else {}))
    monkeypatch.setattr(tool,'collect',changing)
    with pytest.raises(ValueError,match='source changed during'):subject.prepare(root/'drifting','synthetic')
    assert not (root/'drifting/handover_receipt.json').exists()
    assert not (root/'drifting/verification_receipt.json').exists()


@pytest.mark.parametrize('name,raw',[('scripts/private/file.py',b'x'),('scripts/fake.py',b'\x7fELFbinary'),
    ('scripts/secret.py',b'-----BEGIN PRIVATE KEY-----')])
def test_existing_public_bundle_exclusions_are_enforced(name,raw):
    with pytest.raises(ValueError):subject.check_public({name:raw})
