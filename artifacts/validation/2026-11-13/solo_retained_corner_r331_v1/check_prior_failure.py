from pathlib import Path
import arena_autonomy.local_follow as module
import pytest
p=Path(__file__).with_name('local_follow.before.py')
exec(compile(p.read_bytes(),str(p),'exec'),module.__dict__)
raise SystemExit(pytest.main(['-q','tests/test_low_speed_arc_reacquire.py::test_prefix_replay_matches_recorded_decisions']))
