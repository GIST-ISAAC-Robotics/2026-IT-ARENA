"""실제 선택 두 파일 + 합성 collector만 검사한다. 실제 최신 source ZIP은 만들지 않는다."""
import copy
import json
from pathlib import Path
import zipfile

import pytest
import prepare_update_with_steering as subject
from test_prepare_update import physical_inputs

SELECTION=Path(__file__).resolve().parents[1]/'steering_selection.assembled_2026_11_14.json'


@pytest.fixture
def source_fixture(monkeypatch):
    tool=subject.collector()
    files={name:('SYNTHETIC SOURCE '+name+'\n').encode() for name in tool.REQUIRED}
    monkeypatch.setattr(tool,'collect',lambda repo:dict(files))
    monkeypatch.setattr(subject,'collector',lambda:tool)
    monkeypatch.setattr(subject,'baseline',lambda:dict(files={'scripts/old.py':'synthetic-hash'}))
    return files


def test_actual_selection_is_ready_and_does_not_supply_a_motor_board():
    selection=subject.load(SELECTION);files=subject.steering_files(selection)
    assert len(files)==5
    assert subject.check_steering_selection(selection,files)==selection
    assert selection['command_limit_rad']==0.28475436568260193
    assert not {'motor','kp','ki','runtime_port','board_profile'}.intersection(selection)
    zero=json.loads(files[selection['steering_profile']['path']])
    assert (zero['zero_count'],zero['safe_min_count'],zero['safe_max_count'])==(2811,2600,2980)
    assert not zero['hardware_angle_measured']


@pytest.mark.parametrize('select',[False,True])
def test_optional_selection_is_packed_and_validated_without_installing(tmp_path,source_fixture,select):
    output=tmp_path/'source_candidate'
    result=subject.prepare(output,'SYNTHETIC source only',steering_selection=SELECTION if select else None)
    assert result['steering_selection_prepared'] is select
    assert result['install_pending'] and not result['hardware_selection_prepared']
    assert subject.validate(output,against_current=True)==result
    assert not (output/'private').exists()
    with zipfile.ZipFile(output/'runtime_handover_sources.zip') as archive:
        assert (subject.STEERING_SELECTION in archive.namelist()) is select
        if select:
            selection=json.loads(archive.read(subject.STEERING_SELECTION))
            for key in ('steering_profile','vehicle_config'):
                item=selection[key]
                assert subject.sha(archive.read(item['path']))==item['sha256']
            assert b'runtime_installed_profile.py prepare' in archive.read(subject.STEERING_GUIDE)
            assert b'--steering-profile config/hardware/steering_zero.assembled_2026_11_14.json' in archive.read(subject.STEERING_GUIDE)
        assert json.loads(archive.read('handover/required_inputs.json'))['assembly_id'] is None


@pytest.mark.parametrize('bad',['missing','sha','limit','joint'])
def test_selection_cannot_silently_omit_or_mismatch_its_inputs(bad):
    value=subject.load(SELECTION);files=subject.steering_files(value)
    if bad=='missing':del files[value['vehicle_config']['path']]
    elif bad=='sha':value['steering_profile']['sha256']='0'*64
    elif bad=='limit':value['command_limit_rad']=.2
    else:value['simulation_joint_limit_rad']=value['command_limit_rad']
    with pytest.raises(ValueError):subject.check_steering_selection(value,files)


def test_explicit_board_must_match_selection_without_modifying_motor_values(tmp_path,source_fixture):
    inputs=physical_inputs(tmp_path)
    selection=subject.load(SELECTION)
    inputs['steering_profile']=subject.ROOT/selection['steering_profile']['path']
    with pytest.raises(ValueError,match='differ from selected'):
        subject.prepare(tmp_path/'wrong','SYNTHETIC wrong board',steering_selection=SELECTION,**inputs)
    assert not (tmp_path/'wrong').exists()
    board=subject.load(inputs['board_profile']);hardware=subject.load(inputs['hardware_inputs'])
    board['assembly_id']=hardware['assembly_id']=selection['assembly_id']
    before=copy.deepcopy(board)
    board['servo']['command_limit']=selection['command_limit_rad']
    subject.save(inputs['board_profile'],board);subject.save(inputs['hardware_inputs'],hardware)
    output=tmp_path/'matched'
    result=subject.prepare(output,'SYNTHETIC board, real selected steering',steering_selection=SELECTION,**inputs)
    assert result['hardware_selection_prepared'] and result['steering_selection_prepared']
    installed=subject.load(output/'private/installed_profile/runtime_board.json')
    assert {k:v for k,v in installed.items() if k!='servo'}=={k:v for k,v in before.items() if k!='servo'}
    assert subject.validate(output)['ok']
    board['servo']['track']+=.001
    subject.save(inputs['board_profile'],board)
    with pytest.raises(ValueError,match='geometry'):
        subject.prepare(tmp_path/'wrong_geometry','SYNTHETIC wrong geometry',steering_selection=SELECTION,**inputs)
