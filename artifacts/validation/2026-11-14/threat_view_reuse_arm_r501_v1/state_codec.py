"""판단 상태의 원값과 두 곳의 표면 정합 시간 진단을 분리해 보존한다."""
import copy
import dataclasses
import json
import math
from collections import deque
from types import FunctionType, MethodType

import numpy as np
from arena_autonomy.follow_trace import freeze


def state(value, seen=None, path='$'):
    if seen is None:
        seen = {}
    if value is None or isinstance(value, (str, bool, int, float, np.generic, np.ndarray)):
        return freeze(value)
    if isinstance(value, (FunctionType, MethodType)):
        return dict(callable=value.__module__+'.'+value.__qualname__)
    if id(value) in seen:
        return dict(state_ref=seen[id(value)])
    seen[id(value)] = path
    if isinstance(value, dict):
        return {str(k): state(v, seen, path+'.'+str(k)) for k, v in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [state(v, seen, path+'['+str(i)+']') for i, v in enumerate(value)]
    if isinstance(value, (set, frozenset)):
        return sorted((state(v, seen, path+'.set') for v in value), key=lambda v: json.dumps(v, sort_keys=True))
    if dataclasses.is_dataclass(value):
        return {f.name: state(getattr(value, f.name), seen, path+'.'+f.name) for f in dataclasses.fields(value)}
    if hasattr(value, '__dict__') and type(value).__module__.startswith('arena_autonomy'):
        return dict(type=type(value).__qualname__, attributes=state(vars(value), seen, path+'.attributes'))
    raise TypeError('unrecorded state type: '+str(type(value)))


def comparable_state(value):
    """경로가 명시된 cached fit의 processing_ms만 0으로 바꾼 비교 사본."""
    result = copy.deepcopy(value)
    attrs = result.get('attributes', {})
    owners = [attrs]
    normal = attrs.get('normal')
    if isinstance(normal, dict):
        owners.append(normal.get('attributes', {}))
    for owner in owners:
        policy = owner.get('partial_policy')
        cache = policy.get('attributes', {}).get('surface_cache') if isinstance(policy, dict) else None
        if isinstance(cache, list) and len(cache) == 3 and isinstance(cache[2], dict) and 'processing_ms' in cache[2]:
            cost = cache[2]['processing_ms']
            if isinstance(cost, bool) or not isinstance(cost, (int, float)) or not math.isfinite(cost) or cost < 0:
                raise ValueError('invalid cached surface processing cost')
            cache[2]['processing_ms'] = 0.
    return result


def differences(a, b, path='$'):
    if type(a) is not type(b):
        return [dict(path=path, before=a, after=b)]
    if isinstance(a, dict):
        if a.keys() != b.keys():
            return [dict(path=path, before_keys=list(a), after_keys=list(b))]
        return [d for k in a for d in differences(a[k], b[k], path+'.'+k)]
    if isinstance(a, list):
        if len(a) != len(b):
            return [dict(path=path, before_length=len(a), after_length=len(b))]
        return [d for i, (x, y) in enumerate(zip(a, b)) for d in differences(x, y, path+'['+str(i)+']')]
    return [] if a == b else [dict(path=path, before=a, after=b)]
