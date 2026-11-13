"""원 r463 정책을 보존하고 한 helper만 별도 stage에서 batch 계산으로 바꾼다."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

A = Path(__file__).resolve().parent
O = A / 'optimization_v1'
source = A / 'bundle/stage'
target = O / 'stage'
target.mkdir(parents=True, exist_ok=False)
shutil.copytree(source, target, dirs_exist_ok=True)
relative = Path('src/arena_autonomy/arena_autonomy/known_structure.py')
path = target / relative
text = path.read_text(encoding='utf8')
old_sha = hashlib.sha256(path.read_bytes()).hexdigest()
text = text.replace('    from arena_autonomy.rear_fringe_trial import _query_brackets\n', '', 1)
first = text.index('    for radius, angles, lo, hi in _query_brackets(q, origins, directions):')
last = text.index("    return {**result, 'accepted': True", first)
text = text[:first] + '''    # 광선 전체와 각 취득 원점은 유지한다. 최대 32질의의 계산만 배열화한다.
    # 첫 실패 질의와 그 질의의 기존 거절 우선순위는 아래 masks 순서로 보존한다.
    by_id = list(dict.fromkeys(c for c in components.values() if c is not None))
    ids = {c: i for i, c in enumerate(by_id)}
    component_ids = np.full(len(rays), -1, dtype=int)
    for ray, component in components.items():
        if component is not None:
            component_ids[ray] = ids[component]
    available = depths-pad
    for radius, angles, lo, hi in _current_mask_query_batches(q, origins, directions):
        row = np.arange(len(radius))
        low_angles, high_angles = angles[row, lo], angles[row, hi]
        bracket_bad = (np.any(radius <= 1e-9, axis=1) | (low_angles < -1e-10)
            | (high_angles > 1e-10) | (np.abs(lo-hi) > 1)
            | (np.abs(low_angles)+np.abs(high_angles) > .025))
        low_masked, high_masked = status[lo] == 'masked', status[hi] == 'masked'
        low_return, high_return = valid[lo], valid[hi]
        status_bad = (~(low_masked | high_masked) | ~(low_masked | low_return)
            | ~(high_masked | high_return))
        pair_near = ((low_return & (available[lo] <= radius[row, lo]))
            | (high_return & (available[hi] <= radius[row, hi])))
        selected = component_ids[np.where(low_masked, lo, hi)]
        unexplained = selected < 0
        wide, flank_near = np.zeros(len(row), dtype=bool), np.zeros(len(row), dtype=bool)
        reaches, widths = np.zeros(len(row)), np.zeros(len(row))
        # 인접한 두 masked 광선은 반드시 같은 연속 component다. 인접하지 않은
        # 질의는 bracket_bad로 먼저 거절하므로 서로 다른 성분을 합치지 않는다.
        ordered = list(dict.fromkeys(selected.tolist()))
        for index in ordered:
            if index < 0:
                continue
            a, b, angle, origin_span = by_id[index]
            chosen = np.flatnonzero(selected == index)
            reach = np.max(radius[chosen, a-1:b+2], axis=1)
            width = 2*reach*math.sin(angle/2) + origin_span
            reaches[chosen], widths[chosen] = reach, width
            wide[chosen] = (angle >= math.pi/2) | (width > ACCEPTED_HIDDEN_WIDTH_M)
            flank_near[chosen] = ((available[a-1] <= radius[chosen, a-1])
                | (available[b+1] <= radius[chosen, b+1]))
        masks = (bracket_bad, status_bad, pair_near, unexplained, wide, flank_near)
        reasons = ('unbracketed_current_mask_query', 'not_declared_mask_only',
            'near_current_mask_flank', 'unexplained_current_mask_component',
            'wide_current_structure_shadow', 'near_current_mask_flank')
        failed = np.flatnonzero(np.logical_or.reduce(masks))
        if len(failed):
            first_bad = failed[0]
            return reject(next(reason for reason, mask in zip(reasons, masks) if mask[first_bad]))
        for index in ordered:
            a, b, _, _ = by_id[index]
            chosen = np.flatnonzero(selected == index)
            # argmax의 첫 동률 선택과 batch 간 strict > 갱신으로 원 질의 순서 유지.
            best = chosen[np.argmax(widths[chosen])]
            width, reach = float(widths[best]), float(reaches[best])
            key = (a, b)
            old = groups.get(key)
            if old is None or width > old['hidden_width_m']:
                groups[key] = dict(first=a, last=b, hidden_width_m=width,
                    flank_ranges_m=depths[[a-1, b+1]].tolist(), query_reach_m=reach)
''' + text[last:]
position = text.index('\ndef current_mask_shadow(')
helper = '''
def _current_mask_query_batches(queries, origins, directions):
    """전체 광선 bracket을 원 연산/동률 규칙으로 최대 32질의씩 계산한다."""
    for start in range(0, len(queries), 32):
        dx = queries[start:start+32, None, 0]-origins[None, :, 0]
        dy = queries[start:start+32, None, 1]-origins[None, :, 1]
        # 2원소 norm의 multiply/add/sqrt 순서를 유지한다. hypot나 제곱 임계값으로
        # 바꾸지 않으며 dot의 +0은 np.sum([-0., -0.])의 양의 0도 보존한다.
        radius = np.sqrt(dx*dx + dy*dy)
        cross = directions[None, :, 0]*dy-directions[None, :, 1]*dx
        dot = directions[None, :, 0]*dx + directions[None, :, 1]*dy + 0.
        angles = np.arctan2(cross, dot)
        lows = np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf), axis=1)
        highs = np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf), axis=1)
        yield radius, angles, lows, highs

'''
text = text[:position] + helper + text[position:]
ast.parse(text)
path.write_text(text, encoding='utf8', newline='\n')
changes = []
for before in sorted(source.rglob('*')):
    if before.is_file():
        after = target / before.relative_to(source)
        a = hashlib.sha256(before.read_bytes()).hexdigest()
        b = hashlib.sha256(after.read_bytes()).hexdigest()
        if a != b:
            changes.append(dict(path=before.relative_to(source).as_posix(), before=a, after=b))
assert len(changes) == 1 and changes[0]['path'] == relative.as_posix()
(O / 'changes.json').write_text(json.dumps(changes, indent=2))
print(json.dumps(changes))
