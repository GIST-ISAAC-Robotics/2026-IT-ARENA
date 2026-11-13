"""독립 번들의 고정 경로/해시 검사. 장치·ROS·네트워크를 열지 않는다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
POLICY = A.parent/'released_current_r375_v1'
EXPECTED = POLICY/'results_native_v1/comparison.json'
PACKAGES = ('src/arena_autonomy', 'src/arena_vehicle_interface')
CHAIN = (
    'artifacts/validation/2026-11-13/released_current_r375_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_corner_r370_v1/candidate.py',
    'artifacts/validation/2026-11-13/solo_crawl_retry_r352_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_gap_r346_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_low_r343_v1/candidate.py',
    'artifacts/validation/2026-11-12/current_region_policy_r281_v1/candidate.py',
    'artifacts/validation/2026-11-12/retained_route_execution_r266_v1/candidate_policy.py',
)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): value.update(block)
    return value.hexdigest()


def set_paths():
    for relative in reversed(PACKAGES):
        path = str(ROOT/relative)
        if path not in sys.path: sys.path.insert(0, path)


def load_comparison():
    set_paths()
    spec = importlib.util.spec_from_file_location('r382_original_comparison', POLICY/'compare.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_manifest():
    manifest = json.loads((A/'source_manifest.json').read_bytes())
    changed = [name for name, expected in manifest['files'].items()
               if not (ROOT/name).is_file() or digest(ROOT/name) != expected]
    assert not changed, ('bundle source/input mismatch', changed)
    return manifest


def check_imports(compare, backend):
    modules = [compare.policy, compare.policy.prior, compare.policy.prior.prior,
        compare.policy.prior.prior.prior, compare.policy.prior.prior.prior.prior,
        compare.policy.prior.prior.prior.prior.base, compare.policy.core_policy]
    found = [Path(module.__file__).resolve().relative_to(ROOT).as_posix() for module in modules]
    assert found == list(CHAIN), ('candidate ancestry mismatch', found)
    compare.configure(backend)
    packages = {}
    for name, module in list(sys.modules.items()):
        if not name.startswith(('arena_autonomy', 'arena_vehicle_interface', 'candidate_recheck')): continue
        filename = getattr(module, '__file__', None)
        if not filename: continue
        path = Path(filename).resolve()
        assert path.is_relative_to(ROOT), ('external installed module loaded', name, str(path))
        packages[name] = path.relative_to(ROOT).as_posix()
    native = None
    if backend == 'native_v1':
        from arena_autonomy import _numeric_kernels
        path = Path(_numeric_kernels.__file__).resolve()
        assert path.is_relative_to(ROOT/'src/arena_autonomy'), ('native extension outside bundle', str(path))
        assert _numeric_kernels.ABI_VERSION == 1
        assert all(hasattr(_numeric_kernels, name) for name in ('depth', 'masked', 'classes', 'polygon'))
        native = dict(path=path.relative_to(ROOT).as_posix(), sha256=digest(path), abi=1)
    return dict(candidate_chain=found, candidate_hashes={name: digest(ROOT/name) for name in found},
                loaded_modules=packages, native=native)
