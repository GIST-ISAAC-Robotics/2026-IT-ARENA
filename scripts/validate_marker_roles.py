#!/usr/bin/env python3
"""합성 마커 픽셀 → 현행 영상 채택 → 실제 분기 코드의 역할 분리 검사.

ROS 클래스 import는 필요하지만 노드·토픽·시뮬·구동을 만들지 않는다.
하위 차량 control만 대체하고 LocalPursuit.control의 분기 선택은 원 코드를 실행한다.
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import zipfile

import cv2
import numpy as np

from arena_autonomy.core import StartSignal
from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.wall_follow import WallFollow
from arena_autonomy.vision_process import VisionPipeline, recognize

ROOT = Path(__file__).resolve().parents[1]
SOURCE_NAMES = ('scripts/validate_marker_roles.py', 'tests/test_marker_roles.py',
                'src/arena_autonomy/arena_autonomy/core.py',
                'src/arena_autonomy/arena_autonomy/local_pursuit.py',
                'src/arena_autonomy/arena_autonomy/wall_follow.py',
                'src/arena_autonomy/arena_autonomy/vision_process.py',
                'src/arena_autonomy/arena_autonomy/race_start.py')


class ImmediateWorker:
    """동일 recognize 함수만 동기 호출한다. 프로세스 성능 검증용이 아니다."""
    def __init__(self):
        self.result = None

    def submit(self, request, data):
        self.result = recognize(request, data)

    def poll(self):
        result, self.result = self.result, None
        return result


def marker_scene(markers, size=96, rotation=0):
    rgb = np.full((480, 848, 3), 120, np.uint8)
    for index, (dictionary_name, marker_id) in enumerate(markers):
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        make = (cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker')
                else cv2.aruco.drawMarker)
        code = np.rot90(make(dictionary, marker_id, size), rotation)
        margin = size//3
        x, y = 50+index*230, 200
        rgb[y-margin:y+size+margin, x-margin:x+size+margin] = 255
        rgb[y:y+size, x:x+size] = code[:, :, None]
    return rgb


def choose_with_current_node(rgb, previous_side, signal_mode='temporary_3light'):
    worker = ImmediateWorker()
    pipeline = VisionPipeline(worker, signal_mode=signal_mode)
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        data=rgb.tobytes(), height=480, width=848, step=848*3, encoding='rgb8')
    pipeline.offer(message)
    pipeline.pump(1.)  # 새 요청을 보내고, 아래 실제 control에서 결과를 채택한다.
    # super()가 실행되므로 실제 파생 클래스 형식은 유지해야 한다.
    # __init__을 호출하지 않아 ROS Node/context/publisher는 만들지 않는다.
    state = object.__new__(LocalPursuit)
    state.__dict__.update(vision_pipeline=pipeline, last_control_time=0.,
                          now_s=lambda: 1.01, ids=[], signal=StartSignal(),
                          side=previous_side, follow=None,
                          timing=SimpleNamespace(call=lambda name, fn, *args: fn(*args)))
    with patch.object(WallFollow, 'control', lambda self: None):
        LocalPursuit.control(state)
    return dict(ids=state.ids, selected_side=state.side,
                accepted=pipeline.stats['accepted'], fault=pipeline.fault,
                signal_started=state.signal.started)


def scenarios():
    # 공식 등록 상태의 10/12 기록에 있는 ID만 합성한다. 새 공식 상태 조회가 아니다.
    for marker in (1, 7, 11, 14, 49):
        for previous in ('left', 'right'):
            yield dict(name=f'vehicle_4x4_{marker}_{previous}', previous=previous,
                       markers=[('DICT_4X4_50', marker)], size=96, rotation=0,
                       expected_ids=[marker], expected_side=previous)
    for size in (36, 72, 144):
        for rotation in range(4):
            yield dict(name=f'vehicle_7x7_91_px{size}_rot{rotation}', previous='right',
                       markers=[('DICT_7X7_100', 91)], size=size, rotation=rotation,
                       expected_ids=[], expected_side='right')
    for marker in (0, 20, 30, 45):
        for vehicle_dictionary, vehicle_id in (('DICT_4X4_50', 49), ('DICT_7X7_100', 91)):
            side = 'right' if marker == 30 else 'left'
            yield dict(name=f'course_{marker}_vehicle_{vehicle_id}',
                       previous='left' if side == 'right' else 'right',
                       markers=[('DICT_4X4_50', marker), (vehicle_dictionary, vehicle_id)],
                       size=96, rotation=0, expected_side=side,
                       expected_ids=sorted([marker]+([vehicle_id] if vehicle_id == 49 else [])))
    yield dict(name='no_marker_keeps_previous_side', previous='right', markers=[],
               size=96, rotation=0, expected_ids=[], expected_side='right')
    yield dict(name='two_course_ids_current_right_precedence', previous='left',
               markers=[('DICT_4X4_50', 20), ('DICT_4X4_50', 30)],
               size=96, rotation=0, expected_ids=[20, 30], expected_side='right')


def evaluate(case, mode):
    result = choose_with_current_node(marker_scene(case['markers'], case['size'], case['rotation']),
                                      case['previous'], mode)
    matched = (result['ids'] == case['expected_ids'] and result['selected_side'] == case['expected_side']
               and result['accepted'] == 1 and result['fault'] is None and not result['signal_started'])
    return dict(case=case, mode=mode, matched=matched, result=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('new artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    hashes = {name: digest(ROOT/name) for name in SOURCE_NAMES}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for name in SOURCE_NAMES:
            bundle.write(ROOT/name, name)
    cases = [evaluate(case, mode) for mode in ('temporary_3light', 'official_4red_candidate')
             for case in scenarios()]
    unchanged = all(digest(ROOT/name) == value for name, value in hashes.items())
    report = dict(passed=unchanged and all(row['matched'] for row in cases), cases=cases,
                  source_sha256=hashes, sources_unchanged=unchanged, opencv=cv2.__version__,
                  scope='synthetic_pixels_and_current_route_branch_not_vehicle_detection_or_field_validation',
                  ros_nodes_created=False, drive_published=False, detects_7x7_vehicles=False,
                  no_detection_means_no_vehicle=False,
                  limitations=['Known finite synthetic markers only; not arbitrary false-positive immunity.',
                               'Course IDs on vehicles cannot be distinguished by ID-only routing.',
                               'No pose, distance, lighting, motion blur, ROS timing or race-start qualification.'])
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], cases=len(cases), sources_unchanged=unchanged)))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
