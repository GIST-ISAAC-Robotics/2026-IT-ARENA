"""r168 기존 적용 범위와 r170 관측 GUARD 재검사를 명시 시험에서 결합한다."""
import importlib.util
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


previous = load('r171_previous', ROOT/'artifacts/validation/2026-11-12/released_runtime_r168_v1/rebase.py')
guard = load('r171_guard', ROOT/'artifacts/validation/2026-11-12/present_guard_current_r170_v1/current.py')


def supported(output):
    return previous.supported(output) or guard.supported(output)


def rebase(output, submitted, current, scene, history, cfg):
    if previous.supported(output):
        return previous.rebase(output, submitted, current, scene, history, cfg)
    return guard.rebase(output, submitted, current, scene, history, cfg)
