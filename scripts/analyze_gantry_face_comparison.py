"""동일 시야의 PNG/도형 RGB 결과와 검출 후보를 비교합니다. 저장 영상만 읽습니다."""
import argparse
import json
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parents[1]
BASE = REPO / 'artifacts/validation/2026-09-15/official_update'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='face_comparison.json')
    args = parser.parse_args()
    target = (BASE / args.output).resolve()
    if target.parent != BASE.resolve() or target.exists():
        raise ValueError('새 비교 결과 파일명만 허용합니다.')
    reports = {case: json.loads((BASE / case / 'report.json').read_text())
               for case in ('face_compare_pbr', 'face_compare_cells')}
    a, b = reports.values()
    assert a['source_world_sha256'] == b['source_world_sha256']
    assert a['camera'] == b['camera'] and a['commands'] == b['commands']
    assert a['opencv'] == b['opencv']
    for name in a['views']:
        assert a['views'][name]['pose_xyz_rpy'] == b['views'][name]['pose_xyz_rpy']
    result = {'same_source_camera_poses_commands_opencv': True, 'opencv': cv2.__version__,
              'scope': '동일 조건 정지 RGB. 원본 PNG 링크 보존. 재질/도형 동시 변경이므로 원인 완전 분리 아님.',
              'cases': {}, 'diagnostic_id30_150cm': {}}
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for case, report in reports.items():
        views = {name: view for name, view in report['views'].items() if 'expected_id' in view}
        result['cases'][case] = {'capture_complete': report['capture_complete'],
            'cleanup': report['cleanup'],
            'default_detected_positions': sum(v['expected_detected_all_frames'] for v in views.values()),
            'total_positions': len(views),
            'detections': {name: v['expected_detected_all_frames'] for name, v in views.items()}}
        gray = cv2.imread(str(BASE / case / 'id30_150cm.png'), cv2.IMREAD_GRAYSCALE)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
        params = cv2.aruco.DetectorParameters_create()
        params.minMarkerDistanceRate = .02
        loose, loose_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        detail = {'default_ids': [] if ids is None else ids.flatten().tolist(),
                  'default_rejected': [r.reshape(4, 2).tolist() for r in rejected],
                  'rate_002_ids': [] if loose_ids is None else loose_ids.flatten().tolist(),
                  'rate_002_target_corners': []}
        for index, marker_id in enumerate([] if loose_ids is None else loose_ids.flatten()):
            if marker_id == 30:
                quad = loose[index].reshape(4, 2)
                detail['rate_002_target_corners'].append(quad.tolist())
        # 고정 시야에서 판 주변 명암을 수치로만 남깁니다. 이미지를 보정/재생성하지 않습니다.
        # ID 30 1.5 m의 화면상 판은 x395..435, y82..124 부근입니다.
        detail['fixed_pixels_gray_xy'] = {f'{x},{y}': int(gray[y, x]) for x, y in (
            (390, 100), (398, 100), (403, 100), (418, 108))}
        result['diagnostic_id30_150cm'][case] = detail
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
