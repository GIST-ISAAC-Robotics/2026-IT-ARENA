"""GC 대조 요약에서 누락/다른 입력/무효 B를 성공으로 만들지 않는다."""
import copy
import hashlib
from pathlib import Path
import sys
import zipfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_evaluator_gc import comparison, source_snapshot_valid


def row(mode):
    return {'mode': mode, 'outcome_accepted_for_gc_comparison': True,
            'source_sha256': {'a': '123'}, 'runtime_vehicle_sha256': 'vehicle', 'world_normalized_sha256': 'world',
            'meshes_sha256': {'m': 'mesh'}, 'profile': {'path': 'p', 'sha256': 'profile'}, 'spawn_pose': {'x': 0},
            'lateness_ms': {'max': 1 if mode == 'defer' else 12}, 'gc_policy': {'peak_rss_after_kib': 500000}}


def test_native_and_defer_same_input():
    result = comparison([row('native'), row('defer'), row('defer'), row('native')])
    assert result['accepted_bounded_comparison'] and result['replicates'] == {'native': 2, 'defer': 2}


@pytest.mark.parametrize('field', ['source_sha256', 'runtime_vehicle_sha256', 'world_normalized_sha256',
                                   'meshes_sha256', 'profile', 'spawn_pose'])
def test_different_input_cannot_be_accepted(field):
    a, b = row('native'), row('defer')
    b[field] = 'changed'
    assert not comparison([a, b])['accepted_bounded_comparison']


@pytest.mark.parametrize('modes', [[], ['native'], ['defer'], ['defer', 'defer']])
def test_missing_comparison_arm(modes):
    assert not comparison([row(m) for m in modes])['accepted_bounded_comparison']


def test_invalid_run_not_accepted():
    a, b = row('native'), row('defer')
    b['outcome_accepted_for_gc_comparison'] = False
    assert not comparison([a, b])['accepted_bounded_comparison']


def test_source_zip_exact_members_and_hash(tmp_path):
    data = b'old source'
    path = tmp_path/'source.zip'
    with zipfile.ZipFile(path, 'w') as z:
        z.writestr('a.py', data)
    expected = {'a.py': hashlib.sha256(data).hexdigest()}
    assert source_snapshot_valid(path, expected)
    assert not source_snapshot_valid(path, {**expected, 'missing.py': '1'})
    assert not source_snapshot_valid(path, {'a.py': '1'})
    assert not source_snapshot_valid(path, {})
