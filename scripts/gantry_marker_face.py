"""공식 링크를 보존하는 선택형 갠트리 마커 도형 앞면. 진단용이며 기본값 아님."""
import copy
import math
import xml.etree.ElementTree as ET

from experimental_facilities import BLACK, WHITE, box, marker_cells


def add_sdf_marker_faces(track):
    """10 cm 판·7 cm 코드를 과거와 같은 무텍스처 재질로 표현합니다.

    공식 5 mm 판의 +X 면 앞에 흰 면과 검은 셀을 추가하며, 원본 링크·
    collision·pose는 바꾸지 않습니다. 새 앞면은 visual 전용입니다.
    """
    before = [ET.tostring(link) for link in track.findall('link')]
    prepared, report = [], []
    for marker_id in (0, 20, 30, 45):
        name = f'team_marker_face_{marker_id}'
        if track.find(f"link[@name='{name}']") is not None:
            raise ValueError(f'이미 존재하는 앞면: {name}')
        original = track.find(f"link[@name='aruco_{marker_id}']")
        if original is None:
            raise ValueError(f'공식 마커 없음: {marker_id}')
        size = list(map(float, original.findtext('visual/geometry/box/size').split()))
        if len(size) != 3 or not all(math.isclose(a, b, abs_tol=1e-9)
                                       for a, b in zip(size, (.005, .1, .1))):
            raise ValueError(f'예상과 다른 공식 판 크기: {size}')
        link = ET.Element('link', name=name)
        link.append(copy.deepcopy(original.find('pose')))
        box(link, 'white_face', (.0027, 0, 0), (.0002, .1, .1), WHITE)
        cells, cell_size = marker_cells(marker_id), .07 / 6
        for row in range(6):
            for col in range(6):
                if cells[row, col] != 0:
                    continue
                box(link, f'ink_{row}_{col}',
                    (.0029, -.035 + (col + .5) * cell_size,
                     .035 - (row + .5) * cell_size),
                    (.0002, cell_size, cell_size), BLACK)
        prepared.append(link)
        report.append({'id': marker_id, 'link': name, 'board_m': .1, 'code_m': .07,
                       'face_center_local_x_m': .0027, 'ink_center_local_x_m': .0029,
                       'thickness_m': .0002, 'visuals': len(link.findall('visual')),
                       'collision_added': False})
    track.extend(prepared)
    assert [ET.tostring(link) for link in track.findall('link')[:-4]] == before
    return report
