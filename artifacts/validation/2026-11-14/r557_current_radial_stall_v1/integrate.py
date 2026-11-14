"""Root-owned checked stage transfer; preserve original sources and test fixture."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
old = json.loads((A/'stage_manifest.json').read_text())
for name, row in old.items():
    assert sha(ROOT/name) == row['before'], name
for name in old:
    (ROOT/name).write_bytes((A/'stage'/name).read_bytes())
test = ROOT/'tests/test_reacquire_radial_recheck.py'
backup = A/'original/tests/test_reacquire_radial_recheck.py'
backup.parent.mkdir(parents=True, exist_ok=True)
assert not backup.exists()
backup.write_bytes(test.read_bytes())
s = test.read_text(encoding='utf8')
needle = "    candidate = helper.current_wall_candidate(scene, 0., 'left', 0., .1, CONFIG)"
assert s.count(needle) == 1
s = s.replace(needle,
    "    # This fixture specifically exercises the retained radial policy above the crawl band.\n"
    "    submitted['speed_mps'] = .200001\n"
    "    candidate = helper.current_wall_candidate(scene, .200001, 'left', 0., .1, CONFIG)")
test.write_text(s, encoding='utf8', newline='\n')
paths = list(old)+['tests/test_observed_crawl_recheck.py', 'tests/test_reacquire_radial_recheck.py']
(A/'integrated_manifest.json').write_text(json.dumps({name:sha(ROOT/name) for name in paths},
    indent=2)+'\n', encoding='utf8')
print('Transferred 3 policy modules; retained radial fixture and new observed-crawl tests.')
