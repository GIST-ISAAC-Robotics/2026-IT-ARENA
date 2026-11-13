"""저장 비교/무결성 유틸리티. 제어 입력/정책은 변경하지 않는다."""
import hashlib
import json
from pathlib import Path


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path, value):
    with Path(path).open('x', encoding='utf8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def difference(a, b, path='$'):
    if type(a) is not type(b): return dict(path=path, kind='type', expected=repr(a), actual=repr(b))
    if isinstance(a, dict):
        if a.keys() != b.keys(): return dict(path=path, kind='keys', missing=sorted(a.keys()-b.keys()), extra=sorted(b.keys()-a.keys()))
        for key in a:
            found = difference(a[key], b[key], path+'.'+key)
            if found: return found
    elif isinstance(a, list):
        if len(a) != len(b): return dict(path=path, kind='length', expected=len(a), actual=len(b))
        for i, (x, y) in enumerate(zip(a, b)):
            found = difference(x, y, path+'['+str(i)+']')
            if found: return found
    elif a != b: return dict(path=path, kind='value', expected=a, actual=b)
    return None


def raw_recorded(expected):
    result = dict(expected); cap = result.pop('new_plan_deceleration_cap', None)
    if cap is not None: result['speed_mps'] = cap['checked_target_mps']
    return result


def recorded_after_cap(output, expected):
    cap = expected.get('new_plan_deceleration_cap')
    return (output if cap is None else {**output, 'speed_mps': min(output['speed_mps'], cap['speed_mps']),
                                       'new_plan_deceleration_cap': cap})


def diagnostic_summary(output):
    crop = output.get('current_crop_component_retry') or {}
    return dict(speed_mps=output['speed_mps'], reason=output['reason'],
        crop_retry=crop or None,final_reason=crop.get('reason'),final_proof=crop.get('proof'),
        final_region=crop.get('region'),route=output.get('current_route_proof'),
        existing_corner=output.get('current_corner_distance_budget'))


def changed(root, manifest):
    return [name for name, sha in manifest['files'].items()
            if not (root/name).is_file() or digest(root/name) != sha]
