"""모든 출력 차이를 보존한다. 분류는 진단용이며 허용 오차/허가를 바꾸지 않는다."""
import collections
import math


def original_equal(a, b, tolerance=1e-10):
    # frozen follow_trace.equal_output와 동일한 기존 비교 규칙.
    if isinstance(a, (int, float)) and not isinstance(a, bool) and isinstance(b, (int, float)) and not isinstance(b, bool):
        return math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance)
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(original_equal(a[k], b[k], tolerance) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(original_equal(x, y, tolerance) for x, y in zip(a, b))
    return type(a) is type(b) and a == b


def category(path, expected, actual):
    # 미분류 차이는 보수적으로 별도 목록에 남긴다. 이 분류로 판정을 통과시키지 않는다.
    key = str(path[-1]) if path else ''
    if len(path) == 1 and path[0] in (0, 1):
        return 'command_or_measured_state'
    if 'path_points' in path:
        return 'path_coordinate'
    if key in ('queries', 'masked_left', 'masked_right', 'masked_front_samples', 'unobserved'):
        return 'region_count'
    if isinstance(expected, bool) or isinstance(actual, bool):
        return 'permission_or_boolean'
    if key in ('speed_mps', 'steering_rad', 'before_speed_mps', 'after_speed_mps', 'original_target_mps', 'measured_speed_mps'):
        return 'command_or_measured_state'
    if key in ('reason', 'before_reason', 'after_reason', 'mode', 'choice', 'selector_mode', 'selector_choice', 'name'):
        return 'reason_or_selector'
    if key.endswith('_s') or key in ('epoch', 'scan_id'):
        return 'time_or_source'
    return 'other_numeric' if isinstance(expected, (int, float)) and isinstance(actual, (int, float)) else 'other_structure_or_value'


def differences(expected, actual, path=()):
    """JSON 트리의 변경/추가/삭제/길이/타입 차이 전부. 배열을 잘라 버리지 않는다."""
    result = []
    def leaf(kind, old, new, at):
        row = dict(path=list(at), kind=kind, expected=old, actual=new,
                   category=category(at, old, new), existing_comparator_equal=original_equal(old, new))
        if isinstance(old, (float, int)) and not isinstance(old, bool) and isinstance(new, (float, int)) and not isinstance(new, bool):
            if math.isfinite(old) and math.isfinite(new):
                row['absolute_difference'] = abs(new-old)
                row['relative_difference'] = abs(new-old)/max(abs(old), abs(new)) if old or new else 0.
                row['difference_in_max_ulp'] = abs(new-old)/max(math.ulp(float(old)), math.ulp(float(new)))
        result.append(row)
    if type(expected) is not type(actual):
        leaf('type', expected, actual, path)
    elif isinstance(expected, dict):
        for key in sorted(expected.keys() | actual.keys()):
            if key not in actual: leaf('missing', expected[key], None, (*path, key))
            elif key not in expected: leaf('added', None, actual[key], (*path, key))
            else: result.extend(differences(expected[key], actual[key], (*path, key)))
    elif isinstance(expected, list):
        if len(expected) != len(actual):
            leaf('length', len(expected), len(actual), (*path, '<length>'))
        for index in range(max(len(expected), len(actual))):
            if index >= len(actual): leaf('missing', expected[index], None, (*path, index))
            elif index >= len(expected): leaf('added', None, actual[index], (*path, index))
            else: result.extend(differences(expected[index], actual[index], (*path, index)))
    elif expected != actual or (isinstance(expected, float) and expected == 0 and math.copysign(1, expected) != math.copysign(1, actual)):
        leaf('value', expected, actual, path)
    return result


def comparison(expected, actual):
    delta = differences(expected, actual)
    return dict(existing_comparator_equal=original_equal(expected, actual), exact_tree_equal=not delta,
                difference_count=len(delta), categories=dict(collections.Counter(r['category'] for r in delta)),
                differences=delta)


def recorded_result(before, expected):
    cap = expected.get('new_plan_deceleration_cap')
    precondition = None if cap is None else comparison(cap['checked_target_mps'], before['speed_mps'])
    result = before if cap is None else {**before, 'speed_mps': min(before['speed_mps'], cap['speed_mps']),
                                       'new_plan_deceleration_cap': cap}
    return result, precondition
