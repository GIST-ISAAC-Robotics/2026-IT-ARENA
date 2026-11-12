"""r171의 기존 현재 재검사와 RELEASED 현재 차로 유지 재검사를 결합한다."""
import importlib.util
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


previous = load('r174_previous', ROOT/'artifacts/validation/2026-11-12/present_guard_runtime_r171_v1/rebase.py')
released = load('r174_released', ROOT/'artifacts/validation/2026-11-12/released_parallel_r173_v1/current.py')


def supported(output):
    return previous.supported(output) or released.supported(output)


def rebase(output, submitted, current, scene, history, cfg):
    if previous.supported(output):
        return previous.rebase(output, submitted, current, scene, history, cfg)
    return released.rebase(output, submitted, current, scene, history, cfg)
