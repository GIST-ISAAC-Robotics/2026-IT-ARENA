"""Evaluator 인스턴스만 감싼 저장 감사. 전역 함수/정책 교체나 비용 계측 없음."""
import copy
from pathlib import Path
import sys
from support import digest, encoded


class AuditedEvaluator:
    def __init__(self, stage, header):
        from arena_autonomy.race_runtime_policy.evaluator import Evaluator
        self.inner = Evaluator(stage, header)
        self.project = Path(sys.modules[Evaluator.__module__].__file__).resolve().parents[4]
        self.capture = None
        original = self.inner.current_policy
        owner = self
        class Capture:
            core_policy = original.core_policy
            def rebase(self, *args, **kwargs):
                owner.baseline_calls += 1
                result = original.rebase(*args, **kwargs)
                owner.capture = copy.deepcopy(result)
                return result
        self.inner.current_policy = Capture()

    def reset(self, reason): self.inner.reset(reason)
    def close(self): self.inner.close()

    def __call__(self, data):
        from arena_autonomy.follow_trace import freeze
        before = encoded(data); self.capture = None; self.baseline_calls = 0
        output = self.inner(data)
        imports = {}; external = []
        for name, module in sorted(sys.modules.items()):
            if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module, '__file__', None):
                p = Path(module.__file__).resolve()
                if not p.is_relative_to(self.project): external.append(dict(module=name,path=str(p)))
                else: imports[name] = dict(path=p.relative_to(self.project).as_posix(), sha256=digest(p))
        scope = False
        if 'crop_current_input' in data:
            from arena_autonomy.race_runtime_policy.crop_current_scope import same_scan_scope
            scope = same_scan_scope(data, self.inner.follow.config, registered_environment=self.inner.corner_budget)
        return dict(output=output, before_corner=freeze(self.capture), baseline_calls=self.baseline_calls,
                    input_unchanged=before == encoded(data), crop_scope=scope,
                    imports=imports, external_project_imports=external)


def evaluator_factory(stage, header): return AuditedEvaluator(stage, header)
