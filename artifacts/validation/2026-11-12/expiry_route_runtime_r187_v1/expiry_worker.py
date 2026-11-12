"""시험 전용: 폐기 원인을 다음 세대 첫 요청에만 전달한다. 명령은 재사용하지 않는다."""
import ast
from pathlib import Path
from arena_autonomy.fresh_submission import FreshStartDelivery


class Connection:
    def __init__(self, inner, owner):
        self.inner, self.owner = inner, owner

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def send(self, value):
        if isinstance(value, dict) and 'generation' in value:
            hint = self.owner.expiry_generation
            if hint is not None and hint == value['generation']:
                value = {**value, 'reset_reason': 'decision_source_expired'}
                self.owner.expiry_generation = None
        return self.inner.send(value)


class ReasonedWorker:
    def __init__(self, worker):
        self.worker = worker
        self.expiry_generation = None
        worker.connection = Connection(worker.connection, self)

    def __getattr__(self, name):
        return getattr(self.worker, name)

    def invalidate(self):
        self.expiry_generation = None
        self.worker.invalidate()

    def mark_source_expiry(self, item):
        if item and item.get('kind') == 'discarded' and item.get('reason') == 'decision_source_expired':
            self.expiry_generation = self.worker.generation


class ExpiryFreshDelivery(FreshStartDelivery):
    def poll(self, *args, **kwargs):
        item = super().poll(*args, **kwargs)
        self.worker.mark_source_expiry(item)
        return item


def worker_entry(*args):
    # spawn된 자식에서만 동일 계획 후보/최근 참조 비교를 연결한다.
    import importlib.util
    import arena_autonomy.decision_process as process
    here = Path(__file__).resolve().parent
    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module
    plan = load('r187_plan', here.parent/'guard_approach_cap_r181_v1/candidate.py')
    route = load('r187_route', here.parent/'expiry_route_reference_r186_v1/candidate_v2.py')
    plan.install();route.install()
    raw = Path(process.__file__).read_text()
    tree = ast.parse(raw)
    source = ast.get_source_segment(raw,next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_worker'))
    source = source.replace('        generation = None', '        generation = None\n        retained_reference = False')
    before = '                follow.reset()\n'
    after = """                retained_reference = False
                if request.get('reset_reason') == 'decision_source_expired':
                    retained_reference = expiry_reference_reset(follow)
                else:
                    follow.reset()
"""
    assert source.count(before) == 1
    source = source.replace(before, after)
    before = "connection.send(dict(generation=generation, sequence=request['sequence'], size=len(payload),"
    assert source.count(before) == 1
    source = source.replace(before, before+" expiry_reference_retained=retained_reference,")
    namespace = {**vars(process),'expiry_reference_reset':route.reset_reference}
    exec(compile(source,str(__file__)+':worker','exec'),namespace)
    namespace['_worker'](*args)
