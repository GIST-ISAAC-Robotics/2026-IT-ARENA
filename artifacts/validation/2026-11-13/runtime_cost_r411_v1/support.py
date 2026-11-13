"""r411 입력/결과 무결성·기존 상태 checkpoint. 공통 정책을 변경하지 않는다."""
from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DIAGNOSTIC = 'current_corner_distance_budget'


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path, value):
    with Path(path).open('x', encoding='utf8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def verify_manifest():
    manifest = json.loads((ROOT / 'source_manifest.json').read_bytes())
    changed = [n for n, sha in manifest['files'].items()
               if not (ROOT / n).is_file() or digest(ROOT / n) != sha]
    if changed:
        raise RuntimeError(('frozen source/input changed', changed))
    return manifest


def import_receipt():
    modules = {}; external = []
    for name, module in sorted(sys.modules.items()):
        if name.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(module, '__file__', None):
            path = Path(module.__file__).resolve()
            if not path.is_relative_to(ROOT):
                external.append(dict(module=name, path=str(path)))
            else:
                modules[name] = dict(path=path.relative_to(ROOT).as_posix(), sha256=digest(path))
    if external:
        raise RuntimeError(('external project imports', external))
    return modules


def state_value(value):
    # r398 review_support의 checkpoint 계약. 계산 timer 밖에서만 호출한다.
    from arena_autonomy.follow_trace import freeze
    if is_dataclass(value):
        return state_value(asdict(value))
    if isinstance(value, (set, frozenset)):
        return {'__container_type__': type(value).__name__,
                'items': sorted((state_value(v) for v in value), key=encoded)}
    if isinstance(value, dict):
        return {k: state_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [state_value(v) for v in value]
    if hasattr(value, '__dict__'):
        return state_value(vars(value))
    return freeze(value)


def checkpoint(follow):
    normal = getattr(follow, 'normal', follow)
    names = ('path', 'pending', 'path_ref', 'path_source', 'pending_ref', 'recent_path',
             'pending_count', 'release_count', 'last_source', 'last_ref', 'epoch', 'side',
             'path_wall', 'held', 'hold_reason', 'partial_epoch')
    return dict(selector=state_value(normal.pass_selector),
        path=state_value({name: getattr(normal, name) for name in names}),
        handoff=state_value({name: getattr(follow, name) for name in ('mode', 'last_key', 'last_ref', 'reacquire')}))


def statistics(values):
    values = sorted(values); count = len(values)
    if not count:
        return dict(n=0)
    if count == 1:
        return dict(n=1, single_ms=values[0], over_100ms=int(values[0] > 100), over_200ms=int(values[0] > 200))
    def percentile(q):
        at = (count - 1) * q; low = int(at); high = min(low + 1, count - 1)
        return values[low] + (values[high] - values[low]) * (at - low)
    return dict(n=count, p50_ms=percentile(.5), p95_ms=percentile(.95), max_ms=values[-1],
        over_100ms=sum(v > 100 for v in values), over_200ms=sum(v > 200 for v in values))


def current_invariants(before, after, eligible):
    diagnostic = after.get(DIAGNOSTIC) or {}
    if not eligible:
        assert encoded(before) == encoded(after), 'non-target changed'
        return 'non_target'
    assert diagnostic, 'eligible input missing diagnostic'
    assert diagnostic['additional_target_attempts'] in (0, 1)
    assert diagnostic['final_full_checks'] in (0, 1)
    assert diagnostic['unknown_filled'] is diagnostic['source_timestamp_renewed'] is False
    if not diagnostic['applied']:
        assert encoded(before) == encoded({k: v for k, v in after.items() if k != DIAGNOSTIC}), 'rejected candidate changed baseline'
        return 'proposal_rejected' if not diagnostic['proposal']['accepted'] else 'final_check_rejected'
    assert diagnostic['additional_target_attempts'] == diagnostic['final_full_checks'] == 1
    assert diagnostic['current_full_recheck'] is True
    assert 0 < after['speed_mps'] <= diagnostic['proposal']['proposed_target_mps'] <= diagnostic['original_target_mps']
    for key in ('steering_rad', 'reference_s', 'path_source_s', 'original_reference_s', 'path_points'):
        assert encoded(before.get(key)) == encoded(after.get(key)), ('applied identity changed', key)
    return 'applied'
