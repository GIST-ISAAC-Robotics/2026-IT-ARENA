"""저장 입력에서 비교한 작은 정합의 명시 연결·기본값·재관측 경계를 확인한다."""
import ast
import copy
from pathlib import Path

import numpy as np
import pytest

import arena_autonomy.surface_reobservation as module
import arena_autonomy.surface_anchor_trial as anchor_module
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from test_partial_surface_alignment import room


def test_default_is_unchanged_and_selected_policy_keeps_independent_state():
    assert not SurfaceAnchorTrial().small_alignment
    planner=RecoveryPolicy(True,True,True);guard=RecoveryPolicy(True,True,True)
    assert planner.anchor_probe.small_alignment and guard.anchor_probe.small_alignment
    assert planner.anchor_probe is not guard.anchor_probe
    with pytest.raises(ValueError):SurfaceAnchorTrial(small_alignment=1)


def test_connection_preserves_two_fresh_scans_and_lost_object(monkeypatch):
    p,ids=room();calls=[]
    def fit(*args):
        calls.append(1)
        return dict(accepted=True,translation_m=[0.,0.],reason='small_finite_support_consistent')
    monkeypatch.setattr(module,'small_surface_translation',fit)
    monkeypatch.setattr(anchor_module,'transport_points',lambda points,*args:np.array(points,copy=True))
    probe=SurfaceAnchorTrial(extended_surface=True,small_alignment=True)
    probe.start(p[[30,130]],p,ids,1.,.9)
    assert not probe.observe(p,ids,None,1.1,1.)['matched']
    assert not probe.observe(p,ids,None,1.12,1.)['matched']
    assert len(calls)==1
    assert probe.observe(p,ids,None,1.2,1.1)['matched']
    probe.start([[.35,.02]],p,ids,2.,1.9)
    for t in (2.1,2.2):assert not probe.observe(p,ids,None,t,t-.1)['matched']


def test_rejected_fit_is_not_accumulated(monkeypatch):
    p,ids=room();probe=SurfaceAnchorTrial(small_alignment=True)
    probe.start(p[[30]],p,ids,1.,.9);before=copy.deepcopy(probe.anchor)
    monkeypatch.setattr(anchor_module,'transport_points',lambda points,*args:np.array(points,copy=True))
    monkeypatch.setattr(module,'small_surface_translation',lambda *args:dict(accepted=False,reason='weak_improvement'))
    assert not probe.observe(p,ids,None,1.1,1.)['matched']
    np.testing.assert_array_equal(before[0],probe.anchor[0])
    assert before[1:]==probe.anchor[1:]


def test_connected_algorithms_match_reviewed_finite_candidate_ast():
    root=Path(__file__).resolve().parents[1]
    candidate=ast.parse((root/'artifacts/validation/2026-10-28/rotation_reobservation_review_v1/finite_segment_candidate.py').read_text(encoding='utf-8'))
    source=ast.parse(Path(module.__file__).read_text(encoding='utf-8'))
    expected={n.name:n for n in candidate.body if isinstance(n,ast.FunctionDef)}
    actual={n.name:n for n in source.body if isinstance(n,ast.FunctionDef)}
    expected['surface_translation'].name='small_surface_translation'
    for name in ('bounded_solution','common_support_loss','surface_translation'):
        key='small_surface_translation' if name=='surface_translation' else name
        assert ast.dump(expected[name])==ast.dump(actual[key])
