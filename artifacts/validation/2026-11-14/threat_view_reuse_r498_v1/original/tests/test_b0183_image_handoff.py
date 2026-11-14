"""실물 드라이버가 내보낼 RGB/BGR·패딩 경계. CSI/노출/FPS 실측이 아니다."""
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from arena_autonomy.core import image_rgb
from arena_autonomy.vision_process import VisionPipeline, VisionProcess, recognize
from audit_start_signal_layout import light_image
from test_vision_process import WorkerDouble


def rgb_frame(green=False):
    image = np.full((720, 1280, 3), 100, np.uint8)
    image[:480, :848] = light_image(0 if green else 4, green, diameter=12, paired=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    generate = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    marker = generate(dictionary, 49, 120)
    image[120:300, 950:1130] = 255
    image[150:270, 980:1100] = marker[:, :, None]
    return image


def image_message(stamp=1., encoding='rgb8', padding=0, green=False):
    rgb = rgb_frame(green)
    encoded = rgb if encoding == 'rgb8' else rgb[:, :, ::-1]
    # Intentionally non-image bytes in every row's padding. They must not leak
    # into pixels or be shifted to the next row.
    rows = np.full((720, 1280*3+padding), 213, np.uint8)
    rows[:, :1280*3] = encoded.reshape(720, -1)
    sec, nanosec = divmod(round(stamp*1e9), 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
                           height=720, width=1280, step=rows.shape[1], encoding=encoding,
                           data=rows.tobytes())


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
@pytest.mark.parametrize('padding', [0, 64])
def test_rgb_bgr_and_row_padding_preserve_pixels_and_recognition(encoding, padding):
    message = image_message(encoding=encoding, padding=padding)
    before = message.data
    actual = image_rgb(message)
    np.testing.assert_array_equal(actual, rgb_frame())
    reference = image_message()
    pipe = VisionPipeline(WorkerDouble(), signal_mode='official_4red_candidate')
    request = dict(epoch=0, sequence=1, stamp=1., signal=vars(pipe.signal),
        signal_mode=pipe.signal_mode,
        image={k: getattr(message, k) for k in ('height', 'width', 'step', 'encoding')})
    expected = recognize(dict(request, image={k: getattr(reference, k)
                            for k in ('height', 'width', 'step', 'encoding')}), reference.data)
    result = recognize(request, message.data)
    assert result['signal'] == expected['signal']
    assert result['ids'] == expected['ids'] == [49]
    assert message.data == before


@pytest.mark.parametrize('invalid', ['nv12', 'rgba8', 'short_step', 'short_body',
                                     'long_body', 'zero_height', 'zero_width'])
def test_bad_layout_faults_before_work_and_reset_does_not_clear_it(invalid):
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    message = image_message()
    if invalid in ('nv12', 'rgba8'):
        message.encoding = invalid
    elif invalid == 'short_step':
        message.step -= 1
    elif invalid == 'short_body':
        message.data = message.data[:-1]
    elif invalid == 'long_body':
        message.data += b'\x00'
    elif invalid == 'zero_height':
        message.height = 0
    else:
        message.width = 0
    pipe.offer(message, 10.)
    assert pipe.pump(1., 10.) is None
    assert 'Invalid vision image layout' in pipe.fault
    assert not worker.sent and not pipe.signal.started
    pipe.reset()
    assert pipe.fault


def test_renaming_bgr_without_conversion_does_not_preserve_color():
    message = image_message(encoding='bgr8', padding=64)
    message.encoding = 'rgb8'
    assert not np.array_equal(image_rgb(message), rgb_frame())


def test_real_worker_mixed_layouts_preserve_start_state_and_marker_id():
    worker = VisionProcess()
    pipe = VisionPipeline(worker, signal_mode='official_4red_candidate')
    try:
        layouts = [('rgb8', 0), ('bgr8', 64), ('rgb8', 64), ('bgr8', 0), ('bgr8', 64)]
        for index, (encoding, padding) in enumerate(layouts):
            stamp = 1.+index*.1
            pipe.offer(image_message(stamp, encoding, padding, green=index >= 2))
            pipe.pump(stamp)
            deadline = time.monotonic()+4.
            accepted = None
            while accepted is None and not pipe.fault and time.monotonic() < deadline:
                time.sleep(.005)
                accepted = pipe.pump(stamp)
            assert not pipe.fault, pipe.fault
            assert accepted is not None
            assert pipe.ids == [49]
            assert pipe.signal.started is (index == 4)
        assert pipe.stats['accepted'] == 5
    finally:
        closed = worker.close()
    assert closed == {'forced': False, 'exitcode': 0}
    assert not worker.process.is_alive()


def test_shared_capacity_overflow_is_rejected_without_overwriting_buffer():
    worker = VisionProcess(capacity=64)
    try:
        before = bytes(worker.buffer)
        with pytest.raises(ValueError, match='capacity'):
            worker.submit({}, bytes(65))
        assert not worker.busy and bytes(worker.buffer) == before
    finally:
        closed = worker.close()
    assert closed == {'forced': False, 'exitcode': 0}
