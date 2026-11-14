"""실제 전환 객체의 선택·고장 분류와 오프라인 후보 이식 경계를 확인한다."""
import ast
from dataclasses import asdict,replace
import hashlib,json,zipfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
from arena_autonomy.bypass_motion_pause_trial import OverspeedHoldGuide
from test_bypass_handoff_corridor_trial import subject
from test_bypass_handoff import context
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def enabled():
    return BypassHandoffFollow(execution=replace(subject().execution,
        motion_pause_policy='bounded_motion_pause_v1'),allow_simulation_trial=True)


def test_default_off_and_explicit_serialized_selection():
    old=subject();new=enabled()
    assert old.execution.motion_pause_policy=='off' and type(old.bypass.guide) is ForecastGuide
    assert type(new.bypass.guide) is OverspeedHoldGuide
    assert HandoffExecutionConfig.from_mapping(asdict(new.execution))==new.execution
    legacy=asdict(old.execution);del legacy['motion_pause_policy']
    assert HandoffExecutionConfig.from_mapping(legacy)==old.execution


@pytest.mark.parametrize('value',['unknown',True,None])
def test_bad_policy_rejected(value):
    with pytest.raises(ValueError):replace(subject().execution,motion_pause_policy=value)


def test_new_policy_cannot_enable_registered_legacy_entry():
    with pytest.raises(ValueError):HandoffExecutionConfig(motion_pause_policy='bounded_motion_pause_v1')


@pytest.mark.parametrize('mode',['FOLLOW','BYPASS','REACQUIRE'])
@pytest.mark.parametrize('on',[False,True])
@pytest.mark.parametrize('genuine_fault',[False,True])
def test_real_command_keeps_only_active_valid_signed_pause(mode,on,genuine_fault):
    owner=enabled() if on else subject();scan,hist,scene=context(.7);owner.mode=mode
    owner.bypass.guide.path=np.array([[0.,0.],[1.,0.]])
    detail=dict(reason='pass_release_reverse_motion_hold',selector_choice='hold_fault',
        selector_reason='reverse_motion_hold',reverse_motion=True,measured_speed_mps=-.15,
        input_error='missing history' if genuine_fault else None)
    owner.normal.command=lambda *a,**kw:(0.,0.,detail)
    owner.monitor.update=lambda *a:dict(ready=False,observed=False,fresh=True)
    owner.bypass.command=lambda *a,**kw:(0.,.02,dict(reason='entry_reverse_motion_hold',entry_phase='entry'))
    with patch.object(owner.bypass,'reset',wraps=owner.bypass.reset) as reset:
        result=owner.command(scene,hist,-.15,'left',.02,.02,raw_scan=scan)
        retained=on and mode!='FOLLOW' and not genuine_fault
        assert result[0]==0
        assert reset.call_count==(0 if retained else 1)
        assert (owner.bypass.guide.path is not None)==retained


def test_candidate_ast_and_source_history_preserved():
    root=Path(__file__).resolve().parents[1];name='config/tests/g2_motion_pause_r1_sha256.json'
    c=json.loads((root/name).read_text(encoding='utf-8'));sha=lambda b:hashlib.sha256(b).hexdigest()
    assert sha((root/c['parent_contract']).read_bytes())==c['parent_contract_sha256']
    assert sha((root/c['before_snapshot']).read_bytes())==c['before_snapshot_sha256']
    with zipfile.ZipFile(root/c['before_snapshot']) as archive:
        for path,row in c['approved_changes'].items():
            assert sha(archive.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path,newer_than=name)==row['after']
    for path,digest in c['new_dependencies'].items():
        assert m3_revision_digest(path,newer_than=name)==digest
    def node(path,name):
        return ast.dump(next(n for n in ast.parse((root/path).read_text(encoding='utf-8')).body
            if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name))
    runtime='src/arena_autonomy/arena_autonomy/bypass_motion_pause_trial.py'
    for folder,name in [('g2_overspeed_hold_v1','OverspeedHoldGuide'),('g2_motion_pause_handoff_v1','signed_bypass_pause')]:
        assert node(runtime,name)==node('artifacts/validation/2026-10-31/'+folder+'/candidate.py',name)
