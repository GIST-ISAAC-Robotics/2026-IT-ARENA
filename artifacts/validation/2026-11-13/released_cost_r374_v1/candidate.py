"""시험 전용 동일 이동 영역 재사용. 기본 미연결, select 호출 밖에는 보관하지 않는다.

현재 관측·질의·광선 분류·속도 후보·판정은 원 함수를 그대로 쓴다. 같은
select 안에서 입력이 동일한 motion_region의 결과만 복사해 반환한다.
"""
from contextlib import contextmanager
from contextvars import ContextVar
import copy
import math

import numpy as np

from arena_autonomy import pass_release_trial as ps


_SCOPE = ContextVar('r374_motion_region_scope', default=None)


def _signature(scene, speed, target, steering, previous, dt, cfg, lat):
    values = (speed, target, steering, previous, dt)
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        return None
    # motion_queries는 두 속도의 max(abs())만 사용한다. 반올림하지 않는다.
    # steering/previous/dt의 signed zero도 보존하며 불변 scene/config는 동일 객체만 허용한다.
    return (id(scene), id(cfg), id(lat),
            *(float(v).hex() for v in (max(abs(speed), abs(target)), steering, previous, dt)))


def _same(a, b):
    if type(a) is not type(b):
        return False
    if isinstance(a, np.ndarray):
        return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, float):
        return a.hex() == b.hex()
    return a == b


@contextmanager
def enabled(*, observer=None, verify_hits=False):
    """명시 단일 프로세스 실험 연결. 종료/예외에도 두 원 함수를 복구한다.

    verify_hits는 캐시 적중마다 원 함수를 다시 계산해 region/context 비트열을
    대조하는 검증 모드다. 그 모드의 경과시간은 성능값으로 사용하지 않는다.
    """
    original_select = ps.PassReleaseSelector.select
    original_region = ps.motion_region

    def region(scene, speed, target, steering, previous, dt, cfg, lat, *, context=None):
        scope = _SCOPE.get()
        key = _signature(scene, speed, target, steering, previous, dt, cfg, lat)
        if scope is None or key is None:
            return original_region(scene, speed, target, steering, previous, dt, cfg, lat,
                                   context=context)
        scope['calls'] += 1
        cached = scope['cache'].get(key)
        if cached is None:
            saved_context = {}
            answer = original_region(scene, speed, target, steering, previous, dt, cfg, lat,
                                     context=saved_context)
            # 강한 참조로 id 재사용을 막고 반환된 배열/사전과 저장본을 분리한다.
            scope['cache'][key] = (scene, cfg, lat, copy.deepcopy(answer), copy.deepcopy(saved_context))
            scope['misses'] += 1
            if context is not None:
                context.update(saved_context)
            return answer
        _, _, _, answer, saved_context = cached
        scope['hits'] += 1
        scope['saved_queries'] += answer['queries']
        if verify_hits:
            checked_context = {}
            checked = original_region(scene, speed, target, steering, previous, dt, cfg, lat,
                                      context=checked_context)
            if not _same(answer, checked) or not _same(saved_context, checked_context):
                raise AssertionError('r374 cache hit changed full region/context')
            scope['verified_hits'] += 1
        if context is not None:
            context.update(copy.deepcopy(saved_context))
        return copy.deepcopy(answer)

    def select(owner, *args, **kwargs):
        scope = dict(cache={}, calls=0, misses=0, hits=0, saved_queries=0, verified_hits=0)
        token = _SCOPE.set(scope)
        try:
            return original_select(owner, *args, **kwargs)
        finally:
            _SCOPE.reset(token)
            if observer is not None:
                observer({k: v for k, v in scope.items() if k != 'cache'})

    ps.motion_region = region
    ps.PassReleaseSelector.select = select
    try:
        yield
    finally:
        ps.motion_region = original_region
        ps.PassReleaseSelector.select = original_select
