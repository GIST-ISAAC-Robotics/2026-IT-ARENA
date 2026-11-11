import hashlib
import json
from pathlib import Path
import sys
from native_candidate import Kernel
from arena_autonomy import pass_release_trial as pr
from arena_autonomy.follow_trace import equal_output

A = Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'current_tracking_r117_v1'))
import probe


def main():
    library = Path(sys.argv[1])
    kernel = Kernel(library)
    original = pr.depth_support
    results, summaries = {}, {}
    for name, implementation in [('baseline',original),('native',kernel.function_variant())]:
        destination = A/name
        destination.mkdir()
        probe.A = destination
        pr.depth_support = implementation
        try:
            probe.main()
        finally:
            pr.depth_support = original
        results[name] = [json.loads(s) for s in (destination/'details.jsonl').open()]
        summaries[name] = json.loads((destination/'comparison.json').read_bytes())
    assert len(results['baseline']) == len(results['native'])
    result = dict(summaries=summaries,full_output_difference_indices=[i for i,(a,b) in
        enumerate(zip(results['baseline'],results['native'])) if not equal_output(a,b)],
        native_calls=kernel.calls,native_rows=kernel.rows,numpy_fallback_rows=kernel.fallback_rows,
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256((A/'evaluate.cpp').read_bytes()).hexdigest(),
        scope='Isolated PC fixed-input experiment; no runtime code or command changes')
    with (A/'comparison.json').open('x') as f:
        json.dump(result,f,indent=2)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
