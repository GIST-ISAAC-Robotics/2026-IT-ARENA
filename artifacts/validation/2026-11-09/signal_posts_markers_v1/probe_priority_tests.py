"""작업 트리 코드 변경 없이 저장한 후보 사본에 회귀를 실행한다."""
import sys
from pathlib import Path
import pytest
import arena_autonomy.local_follow as module

A=Path(__file__).resolve().parent
exec(compile((A/'local_follow.priority_candidate.py').read_text(), module.__file__, 'exec'), module.__dict__)
raise SystemExit(pytest.main(['-q', str(A/'test_junction_priority_draft.py'),
    'tests/test_boundary_bridge.py', 'tests/test_traffic_boundary_refresh.py']))
