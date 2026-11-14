"""옵션 OFF 불변·검출 공유·세대/소실·중복 크기 혼동. 주행 성능 검사가 아니다."""
import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_autonomy'))
from arena_autonomy.vehicle_markers import VehicleMarkerObserver, participant_ids, summarize_markers
from arena_autonomy.vision_process import VisionPipeline, recognize
from test_vision_process import WorkerDouble, begin, message


def corners(size=40, x=640, y=360):
    return np.array([[x-size/2, y-size/2], [x+size/2, y-size/2],
                     [x+size/2, y+size/2], [x-size/2, y+size/2]])


def rows(size=40, x=640, identity=12):
    return summarize_markers([(identity, corners(size, x))], 1280, 720, (12, 13))


@pytest.mark.parametrize('ids', [(), (0,), (20,), (30,), (45,), (49,), (50,), (-1,), (True,), (12.1,)])
def test_only_explicit_vehicle_ids(ids):
    with pytest.raises(ValueError):
        participant_ids(ids)


def test_ids_are_canonical_and_course_never_included():
    assert participant_ids((13, 12, 12)) == (12, 13)
    assert summarize_markers([(0, corners()), (49, corners()), (12, corners())], 1280, 720, (12,)) == rows()


def test_small_duplicate_vs_conflicting_same_id():
    same = summarize_markers([(12, corners()), (12, corners(42))], 1280, 720, (12,))
    assert same[0]['duplicates'] == 2 and not same[0]['ambiguous']
    for other in (corners(65), corners(40, 850)):
        conflict = summarize_markers([(12, corners()), (12, other)], 1280, 720, (12,))
        assert conflict[0]['ambiguous']


@pytest.mark.parametrize('quad', [corners(5), corners(40, 0), np.full((4, 2), float('nan')),
                                   corners()[[0, 2, 1, 3]]])
def test_bad_geometry_does_not_produce_an_approach(quad):
    assert not summarize_markers([(12, quad)], 1280, 720, (12,))


def test_growth_is_image_evidence_not_metric_distance_or_a_speed_command():
    observer = VehicleMarkerObserver((12,))
    for t, size in [(1., 40), (1.1, 44), (1.2, 48), (1.3, 53)]:
        assert observer.update(t, rows(size), (720, 1280))
    result = observer.snapshot(1.35)
    assert result['fresh'] and not result['controls_speed'] and not result['metric_range_available']
    obs = result['observations'][0]
    assert obs['growing'] and obs['bearing_stable'] and 0 < obs['inverse_expansion_s'] < 2
    assert not result['no_detection_is_clear']


def test_lateral_marker_growth_does_not_imply_same_path():
    observer = VehicleMarkerObserver((12,))
    for i in range(4):
        observer.update(1.+i*.1, rows(40+i*5, 400+i*40), (720, 1280))
    obs = observer.snapshot(1.3)['observations'][0]
    assert obs['growing'] and not obs['bearing_stable']


def test_loss_ambiguity_and_resolution_change_restart_history():
    observer = VehicleMarkerObserver((12,))
    for i in range(3):
        observer.update(1.+i*.1, rows(40+i*5), (720, 1280))
    assert observer.snapshot(1.2)['observations'][0]['samples'] == 3
    observer.update(1.3, [], (720, 1280))
    assert observer.snapshot(1.3)['observations'] == []
    observer.update(1.4, rows(80), (720, 1280))
    assert observer.snapshot(1.4)['observations'][0]['samples'] == 1
    observer.update(1.5, [dict(rows(90)[0], ambiguous=True)], (720, 1280))
    assert not observer.snapshot(1.5)['observations'][0]['growing']
    observer.update(1.6, rows(90), (720, 1280))
    observer.update(1.7, rows(100), (360, 640))
    assert observer.snapshot(1.7)['observations'][0]['samples'] == 1
    assert not observer.snapshot(2.1)['fresh'] and not observer.snapshot(1.6)['fresh']


def test_duplicate_timestamp_cannot_accumulate_or_refresh():
    observer = VehicleMarkerObserver((12,))
    observer.update(1., rows(), (720, 1280))
    assert not observer.update(1., rows(90), (720, 1280))
    assert not observer.update(.9, rows(90), (720, 1280))
    assert observer.snapshot(1.)['observations'][0]['samples'] == 1
    observer.update(2., rows(90), (720, 1280))
    assert observer.snapshot(2.)['observations'][0]['samples'] == 1


def test_off_result_remains_unchanged_and_on_reuses_one_detector(monkeypatch):
    import arena_autonomy.vision_process as module
    calls = []
    def detect(rgb, rate):
        calls.append(rate)
        return [(12, corners(40, 400, 300)), (20, corners(40, 600, 300))]
    monkeypatch.setattr(module, 'marker_observations', detect)
    # OFF keeps the original marker_ids entry and result schema.
    monkeypatch.setattr(module, 'marker_ids', lambda rgb, rate: [20, 12])
    msg = message()
    request = dict(epoch=0, sequence=1, stamp=1., signal=vars(module.StartSignal()),
        image={k: getattr(msg, k) for k in ('height', 'width', 'step', 'encoding')})
    baseline = recognize(request, msg.data)
    assert calls == [] and 'vehicle_markers' not in baseline
    enabled = recognize(dict(request, vehicle_marker_ids=(12,)), msg.data)
    assert calls == [.02]
    assert enabled['ids'] == baseline['ids'] and enabled['signal'] == baseline['signal']
    assert [row['id'] for row in enabled['vehicle_markers']] == [12]


def test_pipeline_rejects_pre_reset_observations_and_boundary_clears_them():
    worker = WorkerDouble()
    pipeline = VisionPipeline(worker, vehicle_marker_ids=(12,))
    begin(pipeline)
    worker.finish()
    worker.result['vehicle_markers'] = rows()
    pipeline.reset()
    assert pipeline.pump(1.1, 10.1) is None
    assert pipeline.vehicle_observer.stamp is None
    begin(pipeline, stamp=2., wall=11.)
    worker.finish()
    worker.result['vehicle_markers'] = rows()
    assert pipeline.pump(2.1, 11.1)
    assert pipeline.vehicle_observer.snapshot(2.1)['observations']
    pipeline.begin_input_boundary(2.2)
    assert pipeline.vehicle_observer.stamp is None
    assert pipeline.vehicle_observer.snapshot(2.2)['observations'] == []


def test_real_aruco_pixels_keep_signal_and_ids_and_produce_growth():
    import cv2
    from arena_autonomy.core import StartSignal
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = (cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker')
            else cv2.aruco.drawMarker)
    observer = VehicleMarkerObserver((12,))
    for i, size in enumerate((36, 42, 48, 54)):
        rgb = np.full((480, 848, 3), 220, dtype=np.uint8)
        image = draw(dictionary, 12, size)
        y, x = 280-size//2, 424-size//2
        rgb[y:y+size, x:x+size] = image[:, :, None]
        request = dict(epoch=0, sequence=i, stamp=1.+i*.1, signal=vars(StartSignal()),
            image=dict(height=480, width=848, step=848*3, encoding='rgb8'))
        off = recognize(request, rgb.tobytes())
        on = recognize(dict(request, vehicle_marker_ids=(12,)), rgb.tobytes())
        assert on['ids'] == off['ids'] == [12]
        assert on['signal'] == off['signal']
        observer.update(request['stamp'], on['vehicle_markers'], (480, 848))
    assert observer.snapshot(1.3)['observations'][0]['growing']
