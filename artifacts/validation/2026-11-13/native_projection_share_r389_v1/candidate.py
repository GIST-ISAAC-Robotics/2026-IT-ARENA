"""단일 replay_call용 offline 연결. 공통 파일/전역 캐시를 만들지 않는다."""
from contextlib import contextmanager
import importlib.util
from pathlib import Path

from arena_autonomy import native_geometry

_path = Path(__file__).resolve().parent/'staged/native_projection.py'
_spec = importlib.util.spec_from_file_location('r389_native_projection', _path)
projection = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(projection)


def same_arrays(left, right):
    return all(a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()
               for a, b in zip(left, right))


@contextmanager
def enabled(*, verify=False):
    """직렬 offline 한 판단만 감싼다. thread/process 운영용 설치 API가 아니다."""
    original_scene, original_points = native_geometry.native_motion_scene, native_geometry.native_motion_points
    if original_scene is not projection.native_motion_scene or original_points is not projection.native_motion_points:
        raise RuntimeError('r389 requires original native geometry; nested/foreign patch is unsupported')
    shared = None
    stats = dict(scene_calls=0, shareable_scenes=0, points_calls=0, shared_points_calls=0,
                 fallback_points_calls=0, verified_points_calls=0)

    def build_scene(scan, transforms, now, config, epoch, mask):
        nonlocal shared
        shared = None
        stats['scene_calls'] += 1
        scene, shared = projection.native_motion_scene_with_projection(scan, transforms, now, config, epoch, mask)
        stats['shareable_scenes'] += shared is not None
        return scene

    def build_points(scan, transforms, now, config, epoch):
        stats['points_calls'] += 1
        hit = shared is not None and shared.matches(scan, transforms, now, config, epoch)
        stats['shared_points_calls' if hit else 'fallback_points_calls'] += 1
        if not hit:
            return original_points(scan, transforms, now, config, epoch)
        result = shared._points()
        if verify:
            expected = original_points(scan, transforms, now, config, epoch)
            if not same_arrays(expected, result):
                raise AssertionError('r389 raw points dtype/shape/bytes differ')
            stats['verified_points_calls'] += 1
        return result

    native_geometry.native_motion_scene, native_geometry.native_motion_points = build_scene, build_points
    try:
        yield stats
    finally:
        native_geometry.native_motion_scene, native_geometry.native_motion_points = original_scene, original_points
        shared = None
