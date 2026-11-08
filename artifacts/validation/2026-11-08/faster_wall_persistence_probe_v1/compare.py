"""r32 고정 입력에서만, 이전 긴 벽과 일치한 현재 반환의 이동 여유를 대조한다."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
from weakref import WeakKeyDictionary

import numpy as np
from candidate import WallReturnMemory
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, freeze, replay_call
from arena_autonomy.moving_pass import MovingPassGuide

HERE = Path(__file__).resolve().parent
TRACE = HERE.parent/'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'


def main():
    old_eval, old_reset, old_pad = (MovingPassGuide.evaluate, MovingPassGuide.reset,
                                    MovingPassGuide._faster_outer_padding)
    states, histories = WeakKeyDictionary(), WeakKeyDictionary()
    counters, changes, sources, seen_sources = Counter(), [], set(), set()

    def evaluate(self, scene, history, *args, **kwargs):
        histories[self] = history
        return old_eval(self, scene, history, *args, **kwargs)

    def reset(self):
        states.pop(self, None)
        histories.pop(self, None)
        return old_reset(self)

    def padding(self, scene, config, side, wall, growth=0.):
        original, evidence = old_pad(self, scene, config, side, wall, growth)
        state = states.setdefault(self, WallReturnMemory())
        comparison = state.compare(scene, config, side, wall, histories.get(self),
                                   self.raw_scan, self.sectors)
        if not comparison['accepted']:
            return original, evidence
        ids = np.asarray([r.source_index for r in scene.rays if r.status == 'return'])
        fixed = .025+config.measurement_padding_m
        selected = np.isin(ids, comparison['current_indices']) & (original > fixed+1e-12)
        if not selected.any():
            return original, evidence
        seen_sources.add(scene.source_first_s)
        sources.add((scene.source_first_s, side))
        result = original.copy()
        result[selected] = fixed
        return result, {**evidence, 'memory_wall_comparison':comparison, 'all_returns_retained':True}

    MovingPassGuide.evaluate, MovingPassGuide.reset, MovingPassGuide._faster_outer_padding = evaluate, reset, padding
    try:
        for line in TRACE.open(encoding='utf8'):
            e = json.loads(line)
            d = e['data']
            if e['kind'] == 'header':
                mc = MotionConfig(**d['motion_config'])
                engine = BypassHandoffFollow(FollowConfig(**d['follow_config']),
                    HandoffExecutionConfig.from_mapping(d['execution_config']), allow_simulation_trial=True)
            elif e['kind'] == 'reset':
                engine.reset()
            elif e['kind'] == 'call':
                before = d['output'][:2]
                actual = freeze(replay_call(engine, d['input'], mc))
                after = actual[:2]
                counters['calls'] += 1
                counters['zero_before'] += before[0] == 0
                counters['zero_after'] += after[0] == 0
                if not np.allclose(before, after, atol=1e-10, rtol=0):
                    counters['commands_changed'] += 1
                    changes.append(dict(t=d['input']['reference_s'], before=before, after=after))
                counters['released'] += before[0] == 0 and after[0] > 0
                counters['new_zero'] += before[0] > 0 and after[0] == 0
    finally:
        MovingPassGuide.evaluate, MovingPassGuide.reset, MovingPassGuide._faster_outer_padding = old_eval, old_reset, old_pad
    out = dict(scope='offline fixed input, no new trajectory, no runtime wiring', counts=dict(counters),
               distinct_sources=len(seen_sources), source_side_pairs=len(sources), changes=changes,
               input_sha256=sha256(TRACE.read_bytes()).hexdigest(),
               source_sha256={p:sha256((HERE/p).read_bytes()).hexdigest() for p in ('compare.py','candidate.py')})
    (HERE/'candidate_result.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(out['counts']), 'distinct_sources', out['distinct_sources'], 'first', changes[:1])


if __name__ == '__main__':
    main()
