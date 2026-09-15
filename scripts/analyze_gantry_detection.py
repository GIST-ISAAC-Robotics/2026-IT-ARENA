#!/usr/bin/env python3
"""보존 RGB 표본만으로 검출 설정 후보를 비교합니다. 운영 인지 코드는 바꾸지 않습니다."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src/arena_autonomy'))
from arena_autonomy.core import StartSignal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', default=['raw_route', 'runtime_final'])
    parser.add_argument('--output', default='detector_probe.json')
    args = parser.parse_args()
    root = REPO / 'artifacts/validation/2026-09-15/official_update'
    output = (root / args.output).resolve()
    if output.parent != root.resolve() or any((root / case).resolve().parent != root.resolve() for case in args.cases):
        raise ValueError('현재 검증 폴더 안의 직접 하위 경로만 허용합니다.')
    if output.exists():
        raise FileExistsError(output)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    report = {'scope': '저장 RGB 표본의 오프라인 후보 비교. 실물·주행·일반화 검증 아님. 운영 검출기 변경 없음.',
              'opencv': cv2.__version__, 'views': {}, 'grid_signal': {}}
    for case in args.cases:
        meta = json.loads((root / case / 'report.json').read_text())
        results = {}
        for name, view in meta['views'].items():
            rgb = cv2.cvtColor(cv2.imread(str(root / case / view['file'])), cv2.COLOR_BGR2RGB)
            if 'expected_signal' in view:
                signal = StartSignal()
                for _ in range(3):
                    signal.update(rgb)
                report['grid_signal'][name] = {'observed': signal.observed, 'started': signal.started,
                                                'red_frames': signal.red_frames}
                continue
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            result = {'expected_id': view['expected_id'], 'candidates': {}}
            for rate in (.01, .02, .05, .1):
                params = cv2.aruco.DetectorParameters_create()
                params.minMarkerDistanceRate = rate
                _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
                result['candidates'][f'minMarkerDistanceRate_{rate}'] = [] if ids is None else ids.flatten().tolist()
            for block in (3, 7, 11, 15, 23, 31, 41, 61):
                params = cv2.aruco.DetectorParameters_create()
                params.adaptiveThreshWinSizeMin = block
                params.adaptiveThreshWinSizeMax = block
                _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
                result['candidates'][f'adaptive_window_{block}'] = [] if ids is None else ids.flatten().tolist()
            results[name] = result
        report['views'][case] = results
    report['candidate_detection_counts'] = {}
    for case, results in report['views'].items():
        keys = next(iter(results.values()))['candidates']
        report['candidate_detection_counts'][case] = {key: sum(v['expected_id'] in v['candidates'][key] for v in results.values()) for key in keys}
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('candidate_detection_counts', 'grid_signal')}, indent=2))


if __name__ == '__main__':
    main()
