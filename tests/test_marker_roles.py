"""현재 마커 역할 분리의 합성 픽셀 회귀. 노드/토픽/구동 없음."""
import pytest

from validate_marker_roles import scenarios, evaluate, marker_scene, choose_with_current_node


@pytest.mark.parametrize('mode', ['temporary_3light', 'official_4red_candidate'])
@pytest.mark.parametrize('case', list(scenarios()), ids=lambda case: case['name'])
def test_registered_synthetic_roles_keep_actual_route_policy(case, mode):
    result = evaluate(case, mode)
    assert result['matched'], result


def test_duplicate_vehicle_ids_do_not_select_course_branch():
    result = choose_with_current_node(marker_scene([('DICT_4X4_50', 49)]*2), 'right')
    assert result['ids'] == [49]
    assert result['selected_side'] == 'right'
    assert result['accepted'] == 1


def test_id_only_routing_cannot_distinguish_course_id_on_vehicle():
    # 이 API에는 표식이 붙은 물체의 정체를 알려 주는 입력이 없다.
    result = choose_with_current_node(marker_scene([('DICT_4X4_50', 30)]), 'left')
    assert result['selected_side'] == 'right'
    # 같은 이미지는 코스 표식이든 차체 표식이든 같은 결과다. 안전 보장이 아니다.
