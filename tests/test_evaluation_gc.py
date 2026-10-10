"""평가 전용 순환 GC 연기: 기본 보존·예외 복원·예산/인자 가드."""
import ast
from contextlib import nullcontext
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from evaluation_gc import EvaluationGC


class FakeGC:
    def __init__(self, enabled=True):
        self.enabled, self.operations = enabled, []

    def isenabled(self):
        return self.enabled

    def enable(self):
        self.enabled = True
        self.operations.append('enable')

    def disable(self):
        self.enabled = False
        self.operations.append('disable')

    def get_threshold(self):
        return (700, 10, 10)

    def get_stats(self):
        return [{'collections': n} for n in (12, 2, 1)]


@pytest.mark.parametrize('mode', ['native', 'defer'])
@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('throws', [True, False])
def test_original_gc_state_restored(mode, enabled, throws):
    collector = FakeGC(enabled)
    policy = EvaluationGC(mode, grid=True, collector=collector, clock=lambda: 0., rss=lambda: 42)
    with pytest.raises(RuntimeError) if throws else nullcontext():
        try:
            policy.start()
            assert collector.enabled == (enabled if mode == 'native' else False)
            policy.check()
            if throws:
                raise RuntimeError('probe or shutdown failure')
        finally:
            policy.restore()
    assert collector.enabled is enabled and policy.record['restored']
    assert policy.record['peak_rss_after_kib'] == 42
    operations = list(collector.operations)
    policy.restore()
    assert collector.operations == operations
    with pytest.raises(RuntimeError):
        policy.start()


@pytest.mark.parametrize('mode,grid', [('defer', False), ('freeze', True), ('', True)])
def test_invalid_option_rejected(mode, grid):
    with pytest.raises(ValueError):
        EvaluationGC(mode, grid=grid)


@pytest.mark.parametrize('fault', ['time', 'memory', 'reenabled'])
def test_budget_and_unexpected_reenable(fault):
    now, rss = [0.], [42]
    collector = FakeGC()
    policy = EvaluationGC('defer', grid=True, collector=collector, clock=lambda: now[0], rss=lambda: rss[0])
    policy.start()
    now[0] = 601 if fault == 'time' else 1.
    if fault == 'memory':
        rss[0] = 2*1024*1024+1
    if fault == 'reenabled':
        collector.enable()
    try:
        with pytest.raises(RuntimeError):
            policy.check()
    finally:
        policy.restore()
    assert collector.enabled


def test_restore_even_if_diagnostics_raise():
    rss = [42]
    def memory():
        if rss[0] is None:
            raise OSError('cannot read RSS')
        return rss[0]
    collector = FakeGC()
    policy = EvaluationGC('defer', grid=True, collector=collector, rss=memory)
    policy.start()
    rss[0] = None
    with pytest.raises(OSError):
        policy.restore()
    assert collector.enabled and policy.record['restored']


def test_real_probe_restores_after_cleanup_in_finally():
    tree = ast.parse((ROOT/'scripts/probe_braking_contact.py').read_text(encoding='utf-8'))
    final = next(n for n in ast.walk(tree) if isinstance(n, ast.Try)
                 and any('gc_policy.restore()' in ast.unparse(s) for s in n.finalbody)
                 and 'stop_launch(process)' in '\n'.join(ast.unparse(s) for s in n.body))
    assert 'publisher.publish(final_command)' in ast.unparse(final)
    assert 'gc_policy.restore()' in ast.unparse(final.finalbody[0])


def test_disable_precedes_rss_import_and_diagnostic_allocations():
    collector = FakeGC()
    def rss():
        assert not collector.isenabled()
        return 42
    policy = EvaluationGC('defer', grid=True, collector=collector, rss=rss)
    try:
        policy.start()
    finally:
        policy.restore()
    assert policy.record['collections_before'] == policy.record['collections_after']


def test_start_failure_after_disable_restores():
    collector = FakeGC()
    def rss():
        raise OSError('RSS failure')
    policy = EvaluationGC('defer', grid=True, collector=collector, rss=rss)
    try:
        with pytest.raises(OSError):
            policy.start()
    finally:
        with pytest.raises(OSError):
            policy.restore()
    assert collector.isenabled()

