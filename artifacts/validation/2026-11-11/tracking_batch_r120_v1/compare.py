"""원본 후보와 배치 512/1024를 같은 센서 궤적으로 대조. 실행 연결 없음."""
import hashlib
import json
from pathlib import Path
import sys
from variants import depth_variant, border_variant
from arena_autonomy import pass_release_trial as pr, m3_execution_trial as mx
from arena_autonomy.follow_trace import equal_output

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A.parent/'current_tracking_r117_v1'))
import probe


def main():
    files = [probe.SOURCE, Path(pr.__file__), Path(mx.__file__)]
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    before = {str(p):digest(p) for p in files}
    original_depth, original_border = pr.depth_support, mx.path_observation_queries
    outputs = {}
    summaries = {}
    for name, chunk, border in [('baseline',128,False),('chunk512',512,False),
                                ('chunk1024',1024,False),('batch512_border',512,True)]:
        destination = A/name
        destination.mkdir()
        pr.depth_support = original_depth if chunk == 128 else depth_variant(chunk)
        mx.path_observation_queries = border_variant() if border else original_border
        probe.A = destination
        try:
            probe.main()
        finally:
            pr.depth_support, mx.path_observation_queries = original_depth, original_border
        outputs[name] = [json.loads(s) for s in (destination/'details.jsonl').open()]
        summaries[name] = json.loads((destination/'comparison.json').read_bytes())
    expected = [json.loads(s) for s in (A.parent/'current_tracking_r117_v1/details.jsonl').open()]
    differences = {name:[i for i,(a,b) in enumerate(zip(outputs['baseline'], rows)) if not equal_output(a,b)]
                   for name,rows in outputs.items()}
    assert all(len(rows)==len(expected) for rows in outputs.values())
    result = dict(summaries=summaries,full_output_difference_indices=differences,
        baseline_prior_differences=[i for i,(a,b) in enumerate(zip(expected,outputs['baseline'])) if not equal_output(a,b)],
        source_unchanged=before=={str(p):digest(p) for p in files},
        scope='PC saved inputs only; no runtime code change or closed-loop run')
    with (A/'comparison.json').open('x') as f:
        json.dump(result,f,indent=2)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
