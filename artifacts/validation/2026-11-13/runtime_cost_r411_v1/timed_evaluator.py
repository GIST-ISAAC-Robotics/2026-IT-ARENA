"""공통 Evaluator/Lane용 얇은 계측 wrapper. 전역 함수를 교체하지 않는다."""
import time
from support import checkpoint, encoded, import_receipt


class TimedEvaluator:
    def __init__(self, stage, header):
        from arena_autonomy.race_runtime_policy.evaluator import Evaluator
        self.inner = Evaluator(stage, header)
        self.stage = stage
        self.imports = None

    def reset(self, reason):
        self.inner.reset(reason)

    def close(self):
        self.inner.close()

    def __call__(self, data):
        before = encoded(data)
        began = time.perf_counter_ns()
        value = self.inner(data)
        elapsed_ms = (time.perf_counter_ns() - began) / 1e6
        assert before == encoded(data), 'evaluator mutated input'
        eligible = None
        if self.stage == 'current' and not self.inner.corner_budget:
            from arena_autonomy.race_runtime_policy.corner_candidate import eligible as is_eligible
            eligible = is_eligible(value, data['output'], data['current'], self.inner.follow.config, True)
        # 지연 import가 추가되므로 매 호출 경로를 확인한다. 해시/상태비용은 timer 밖이다.
        # 동일 파일의 반복 해시는 피하고 새 모듈이 생긴 때만 receipt를 갱신한다.
        import sys
        names = tuple(n for n in sys.modules if n.startswith(('arena_autonomy', 'arena_vehicle_interface')))
        if self.imports is None or names != self.imports[0]:
            self.imports = (names, import_receipt())
        return dict(output=value, evaluator_ms=elapsed_ms, input_unchanged=True, eligible=eligible,
            state=checkpoint(self.inner.follow) if self.stage == 'plan' else None,
            imports=self.imports[1])


def evaluator_factory(stage, header):
    return TimedEvaluator(stage, header)
