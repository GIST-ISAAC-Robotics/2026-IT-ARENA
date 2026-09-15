"""OpenCV 4.6.0의 제한된 정적 함수만 별도 진단에 추출합니다. 설치 라이브러리는 불변."""
import hashlib
import json
from pathlib import Path
import urllib.request

REPO = Path(__file__).resolve().parents[1]
DEST = REPO/'build/aruco_trace_20260915'
URL = 'https://raw.githubusercontent.com/opencv/opencv_contrib/4.6.0/modules/aruco/src/aruco.cpp'


def function(source, name):
    begin = source.index('static ', source.rfind('/**', 0, source.index(name+'(')))
    brace = source.index('{', source.index(name+'(', begin))
    depth = 1
    end = brace+1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


def main():
    DEST.mkdir(exist_ok=True, parents=True)
    source_file = DEST/'aruco_4.6.0_original.cpp'
    if not source_file.exists():
        source_file.write_bytes(urllib.request.urlopen(URL, timeout=30).read())
    src = source_file.read_text()
    names = ['_threshold', '_findMarkerContours', '_reorderCandidatesCorners', 'alignContourOrder',
             '_filterTooCloseCandidates', '_detectInitialCandidates', '_extractBits',
             '_getBorderErrors', '_identifyOneCandidate']
    fragments = [function(src, name) for name in names]
    group_index = names.index('_filterTooCloseCandidates')
    fragments[group_index] = fragments[group_index].replace(
        '    // save possible candidates\n    candidatesSetOut.clear();',
        '    trace_groups = groupedCandidates;\n    // save possible candidates\n    candidatesSetOut.clear();')
    assert 'trace_groups = groupedCandidates;' in fragments[group_index]
    license_text = src[:src.index('#include')]
    (DEST/'trace_functions.hpp').write_text(license_text+'\n'+
        '#include <opencv2/opencv.hpp>\n#include <opencv2/aruco.hpp>\nusing namespace cv;\nusing namespace cv::aruco;\nusing namespace std;\nvector<vector<unsigned int>> trace_groups;\n'+
        '\n\n'.join(fragments)+'\n')
    print(json.dumps({'source': URL, 'sha256': hashlib.sha256(source_file.read_bytes()).hexdigest(),
                      'functions': names, 'output': str(DEST)}))


if __name__ == '__main__':
    main()
