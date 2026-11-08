import json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
result=json.loads((HERE/'replay_fragment_on_result.json').read_text())
sys.path.insert(0,str(Path(result['tree'])/'src/arena_autonomy'))
import arena_autonomy.local_follow as lf
assert lf.__file__.startswith(result['tree']),lf.__file__
import pytest
raise SystemExit(pytest.main(['-q','--tb=short',str(HERE/'test_current_wall_candidate.py'),str(HERE/'test_fragment_candidate.py')]))
