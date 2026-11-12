import importlib.util
import json
from pathlib import Path

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('r174_test_rebase', A/'rebase.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)


def test_dispatch_adds_only_released_parallel_and_preserves_previous_scope():
    trace = A.parent/'present_guard_runtime_r171_v1/two_actor_run1/run1/follow_decisions.jsonl'
    old_count = new_count = 0
    for row in map(json.loads, trace.open()):
        if row['kind'] != 'call': continue
        output = row['data']['output']
        old = candidate.previous.supported(output)
        new = candidate.released.supported(output)
        assert not (old and new)
        assert candidate.supported(output) == (old or new)
        old_count += int(old); new_count += int(new)
    assert old_count > 0 and new_count == 166
