"""C++ 후보가 실제 전체 판단 비용에도 유효한지 같은 저장 입력으로 비교한다."""
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A.parent/'native_depth_r121_v1'))
from native_candidate import Kernel
from arena_autonomy import observation_support as obs
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call

SOURCE = A.parent/'fresh_submission_r114_v1/two_actor_run1/run1/follow_decisions.jsonl'


def run(events, destination):
    follow, motion = make_controller(events[0]['data'])
    durations, outputs, prior = [], [], []
    for row in events[1:]:
        if row['kind'] == 'reset':
            follow.reset()
        elif row['kind'] == 'call':
            began = time.perf_counter()
            output = replay_call(follow, row['data']['input'], motion)
            durations.append(1000*(time.perf_counter()-began))
            frozen = freeze(output)
            outputs.append(frozen)
            if not equal_output(comparable_decision(frozen), comparable_decision(row['data']['output'])):
                prior.append(row['seq'])
    destination.mkdir()
    with (destination/'outputs.jsonl').open('x') as stream:
        for output in outputs:
            stream.write(json.dumps(output)+'\n')
    with (destination/'durations.json').open('x') as stream:
        json.dump(durations, stream)
    return outputs, dict(calls=len(outputs), prior_difference_sequences=prior,
        cost_ms=dict(p50=float(np.percentile(durations,50)), p95=float(np.percentile(durations,95)),
            maximum=max(durations), over_100ms=sum(x>100 for x in durations)))


def main():
    library = Path(sys.argv[1]).resolve()
    target = Path(sys.argv[2]).resolve()
    target.mkdir(parents=True, exist_ok=False)
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    events = [json.loads(line) for line in SOURCE.open()]
    kernel = Kernel(library)
    original = obs.depth_support
    baseline, before = run(events, target/'baseline')
    # baseline이 실제 사용한 모듈을 모두 로드한 다음 같은 함수 참조만 교체한다.
    # monkeypatch는 이 격리 계측 프로세스에만 존재한다.
    modules = [module for name,module in tuple(sys.modules.items())
               if name.startswith('arena_autonomy.') and getattr(module,'depth_support',None) is original]
    candidate = kernel.function_variant()
    for module in modules:
        module.depth_support = candidate
    try:
        changed, after = run(events, target/'native')
    finally:
        for module in modules:
            module.depth_support = original
    assert len(baseline) == len(changed)
    report = dict(baseline=before,native=after,
        output_difference_indices=[i for i,(a,b) in enumerate(zip(baseline,changed)) if not equal_output(a,b)],
        command_difference_indices=[i for i,(a,b) in enumerate(zip(baseline,changed)) if not equal_output(a[:2],b[:2])],
        patched_modules=[module.__name__ for module in modules],
        kernel_calls=kernel.calls,kernel_rows=kernel.rows,numpy_fallback_rows=kernel.fallback_rows,
        source_sha256=digest,source_unchanged=hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest,
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        scope='Actual planner fixed-input comparison; restore and projection included, ROS/guard/vision excluded')
    with (target/'comparison.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report))


if __name__ == '__main__':
    main()
