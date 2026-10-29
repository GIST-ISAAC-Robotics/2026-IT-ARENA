"""기록된 같은 명령의 후반부 허용 폭 민감도. 실제 출력/상태/기준은 변경하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import pass_release_trial as selector
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe, _bounded_nonreturn
from arena_autonomy.observation_support import source_valid
from arena_autonomy.follow_trace import replay_call, thaw, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig


def sensitivity(scene, raw_scan, queries, padding, length, width, limit):
    """1/3/5/10 mm 비교 전용. 확대 허용을 운영 정책이나 센서 오차 보장으로 쓰지 않는다."""
    if limit not in (.001, .003, .005, .010):
        raise ValueError('only registered sensitivity limits')
    result = dict(accepted=False, reason='invalid_input', unknown_filled=False,
        residual_hidden_object_risk=True, maximum_exterior_m=limit,
        maximum_gap_rad=math.radians(3), minimum_neighbor_range_m=.6)
    if raw_scan is None:
        return {**result, 'reason':'raw_scan_required'}
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = np.asarray(padding, dtype=float)
    if not len(q) or not np.all(np.isfinite(q)) or not source_valid(scene):
        return {**result, 'reason':'invalid_scene_or_queries'}
    if pad.shape != (len(scene.rays),) or not np.all(np.isfinite(pad)) or np.any(pad < 0):
        return {**result, 'reason':'invalid_padding'}
    exterior = np.abs(q[:, 1])-width/2
    result.update(queries=len(q), exterior_max_m=float(exterior.max()),
                  longitudinal_min_m=float(q[:, 0].min()), longitudinal_max_m=float(q[:, 0].max()))
    if (np.any(q[:, 0] < -length/2) or np.any(q[:, 0] > 0)
            or np.any(exterior <= 0) or np.any(exterior > limit)):
        return {**result, 'reason':'outside_rear_fringe'}
    # 이웃 반환 거리/취득 시차 팽창/가까운 점유/3도 공백/원시 일치 검사는 기존 함수 그대로다.
    return _bounded_nonreturn(scene, raw_scan, q, pad, result)


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start', type=float, default=35.)
    parser.add_argument('--end', type=float, default=49.)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = sha(args.trace)
    sources = {str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),
        ROOT/'src/arena_autonomy/arena_autonomy/rear_fringe_trial.py',
        ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py']}
    original = selector.bounded_rear_fringe
    current = {}
    rows = []
    counts = Counter()
    comparisons = []

    def observed(*positional):
        result = original(*positional)
        if args.start <= current['t'] <= args.end:
            answers = {str(limit):sensitivity(*positional, limit) for limit in (.001,.003,.005,.010)}
            # 1 mm 복제의 관측/판정이 기존과 같은지 먼저 확인한다.
            ref = answers['0.001']
            assert result['accepted'] == ref['accepted'] and result['reason'] == ref['reason']
            comparisons.append(dict(original=result, alternatives=answers))
        return result  # 다른 폭의 결과를 실제 선택기에 전달하지 않는다.

    selector.bounded_rear_fringe = observed
    try:
        with args.trace.open(encoding='utf-8') as stream:
            header = json.loads(next(stream))['data']
            ctl = LocalFollow(FollowConfig(**header['follow_config']))
            motion = MotionConfig(**header['motion_config'])
            for line in stream:
                event = json.loads(line)
                if event['kind'] == 'reset':
                    ctl.reset()
                if event['kind'] != 'call':
                    continue
                inputs = thaw(event['data']['input'])
                current['t'] = inputs['reference_s']
                if current['t'] > args.end:
                    break
                comparisons.clear()
                out = replay_call(ctl, inputs, motion)
                assert equal_output(comparable_decision(freeze(out)),
                                    comparable_decision(event['data']['output']))
                counts['matched_prefix_calls'] += 1
                if current['t'] < args.start:
                    continue
                detail = out[2]
                counts['window_calls'] += 1
                if comparisons:
                    rows.append(dict(seq=event['seq'], t=current['t'], command=list(out[:2]),
                        selector_reason=detail.get('selector_reason'), mode=detail.get('selector_mode'),
                        region=detail.get('selector_region'), checks=list(comparisons)))
                    for item in comparisons:
                        for limit, answer in item['alternatives'].items():
                            counts[limit+':'+answer['reason']] += 1
    finally:
        selector.bounded_rear_fringe = original
    result = dict(scope='geometry sensitivity only; same original command/state; no driving or safe-output claim',
        window_s=[args.start,args.end], counts=dict(counts), rows=rows,
        input_sha256=before, input_unchanged=sha(args.trace)==before,
        source_sha256=sources, source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','source_sha256')},ensure_ascii=True))


if __name__ == '__main__':
    main()
