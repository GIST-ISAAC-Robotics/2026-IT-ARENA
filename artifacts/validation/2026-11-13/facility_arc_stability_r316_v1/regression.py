import sys
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(R/'tests')]
import candidate,pytest
candidate.install()
try:
    code=pytest.main(['-q','tests/test_break_adjacent_occluders.py','tests/test_occluder_prefilter_equivalence.py',
        'tests/test_signal_posts.py','tests/test_tracked_wall_occluder.py','tests/test_fragmented_wall_hypothesis.py',
        'tests/test_local_follow.py'])
finally:candidate.uninstall()
raise SystemExit(code)
