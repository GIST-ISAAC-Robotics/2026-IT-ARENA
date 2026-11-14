"""현재 가림 점유 kernel의 경계·명시 선택·오래된 binary 연결 계약."""
import numpy as np
import pytest

from arena_autonomy import numeric_kernels as backend


@pytest.fixture(autouse=True)
def reset():
    backend.configure('python')
    yield
    backend.configure('python')


@pytest.fixture
def extension():
    native = pytest.importorskip('arena_autonomy._numeric_kernels',
        reason='명시 native 후보의 컴파일 모듈 필요')
    if not hasattr(native, 'current_mask_occupied'):
        pytest.fail('native current mask occupancy kernel missing; rebuild arena_autonomy',
            pytrace=False)
    backend.configure('native_v1')
    return native


def test_python_selection_keeps_numpy_fallback():
    assert backend._backend is None
    assert backend.current_mask_occupied(None, None, None) is None


def test_explicit_native_rejects_old_binary_without_entrypoint(monkeypatch):
    monkeypatch.setattr(backend, '_backend', object())
    with pytest.raises(RuntimeError, match='occupancy kernel missing; rebuild arena_autonomy'):
        backend.current_mask_occupied(None, None, None)


@pytest.mark.parametrize('padding,occupied', [
    (np.nextafter(5., -np.inf), False),
    (5., True),
    (np.nextafter(5., np.inf), True),
])
def test_inclusive_distance_boundary_and_input_immutability(extension, padding, occupied):
    queries = np.array([[5., 0.]])
    hits = np.array([[0., 0.]])
    pads = np.array([padding])
    before = [value.copy() for value in (queries, hits, pads)]
    assert backend.current_mask_occupied(queries, hits, pads) is occupied
    for value, original in zip((queries, hits, pads), before):
        np.testing.assert_array_equal(value, original)


@pytest.mark.parametrize('empty', ['queries', 'hits'])
def test_empty_query_or_return_set_has_no_occupancy(extension, empty):
    queries = np.empty((0, 2)) if empty == 'queries' else np.zeros((1, 2))
    hits = np.empty((0, 2)) if empty == 'hits' else np.zeros((1, 2))
    assert backend.current_mask_occupied(queries, hits, np.zeros(len(hits))) is False


@pytest.mark.parametrize('invalid', ['type', 'length', 'readonly'])
def test_extension_rejects_invalid_buffers_before_read(extension, invalid):
    queries = np.zeros((2, 2))
    hits = np.zeros((3, 2))
    pads = np.zeros(3)
    result = np.empty(1, dtype=np.uint8)
    if invalid == 'type':
        queries = queries.astype(np.float32)
    elif invalid == 'length':
        pads = np.zeros(2)
    else:
        result.flags.writeable = False
    with pytest.raises((ValueError, BufferError)):
        extension.current_mask_occupied(2, 3, queries, hits, pads, result)
