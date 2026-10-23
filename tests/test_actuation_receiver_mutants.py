"""변이체 주입 위치가 실제 후보와 일치하는지 컴파일 전에 검사한다."""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import check_actuation_receiver_mutants as mutations


@pytest.mark.parametrize('name,edits', mutations.MUTANTS, ids=[row[0] for row in mutations.MUTANTS])
def test_receiver_mutation_has_exactly_one_anchor(name, edits):
    original = mutations.HEADER.read_text(encoding='utf-8')
    mutated = mutations.mutated_source(original, edits)
    assert mutated != original, name


@pytest.mark.parametrize('text,count', [('unrelated text', 0), ('anchor anchor', 2)])
def test_missing_or_duplicate_anchor_is_a_harness_error(text, count):
    with pytest.raises(ValueError, match=f'found {count}'):
        mutations.mutated_source(text, [('anchor', 'replacement')])
