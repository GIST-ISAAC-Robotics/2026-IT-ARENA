"""평가 프로세스의 교체 참조 복구와 원래 기록 우선 확인."""
import pytest

import probe_corridor_roles as probe


def test_failed_baseline_cannot_start_counterfactual(monkeypatch):
    old = probe.corridor_observer.corridor_evidence
    monkeypatch.setattr(probe, 'events', lambda path: iter(()))
    monkeypatch.setattr(probe, 'replay_events', lambda data: {'passed': False})
    with pytest.raises(ValueError, match='original output replay'):
        probe.compare(None)
    assert probe.corridor_observer.corridor_evidence is old


def test_exception_always_restores_original_reference(monkeypatch):
    old = probe.corridor_observer.corridor_evidence
    calls = []
    def events(path):
        calls.append(True)
        if len(calls) == 1:
            return iter(())
        assert probe.corridor_observer.corridor_evidence is probe.role_aware_evidence
        raise RuntimeError('read failure')
    monkeypatch.setattr(probe, 'events', events)
    monkeypatch.setattr(probe, 'replay_events', lambda data: {'passed': True, 'matched': 1})
    with pytest.raises(RuntimeError, match='read failure'):
        probe.compare(None)
    assert probe.corridor_observer.corridor_evidence is old


def test_completed_shadow_restores_reference(monkeypatch):
    old = probe.corridor_observer.corridor_evidence
    monkeypatch.setattr(probe, 'events', lambda path: iter(()))
    monkeypatch.setattr(probe, 'replay_events', lambda data: {'passed': True, 'matched': 0})
    result = probe.compare(None)
    assert result['function_restored'] and result['calls'] == 0
    assert probe.corridor_observer.corridor_evidence is old
    assert not result['runtime_files_changed'] and not result['independent_safety_verified']
