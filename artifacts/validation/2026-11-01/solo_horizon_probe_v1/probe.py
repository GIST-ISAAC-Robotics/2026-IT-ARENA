"""벽 호길이 탐색 예산과 차체 경로 길이의 차이. 기존 출력은 그대로 재현한다."""
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_events, freeze
from arena_autonomy.lidar_motion import MotionUnavailable

original = module.local_path
current = None
config = None
records = []


def observe(points, side, offset=.425, horizon=1.8):
    result = original(points, side, offset, horizon)
    if current is None or result[0] is None:
        return result
    inp = current['input']; target, _, detail = current['output']
    if (target < 1.8 or detail.get('reason') != 'local_follow'
            or not detail.get('open_route_speed_enabled') or not detail.get('path_points')):
        return result
    path = result[0]; expected = np.asarray(detail['path_points'])
    if path.shape != expected.shape or not np.allclose(path, expected, atol=1e-10, rtol=0.):
        return result
    # 탐색 진단일 뿐 채택 설정이 아니다. 평행 원호의 90도 회전에서 생기는
    # offset*pi/2 길이 차이만큼 추가 벽을 조회한다. 없는 반환은 생성하지 않는다.
    extra = abs(offset)*math.pi/2
    candidate, info = original(points, side, offset, horizon+extra)
    row = dict(t=inp['reference_s'], measured_mps=inp['speed_mps'], recorded_target_mps=target,
               side=side, requested_wall_horizon_m=horizon, extra_wall_search_m=extra,
               original_wall_ahead_m=result[1].get('boundary_observed_ahead_m'),
               expanded_wall_ahead_m=info.get('boundary_observed_ahead_m'), expanded_reason=info.get('reason'),
               recorded_path_ahead_m=detail['observed_path_ahead_m'])
    if candidate is not None:
        try:
            age=max(inp['reference_s']-detail['source_first_s'],detail['boundary_age_s'])
            before=module.pursue_path(path, inp['speed_mps'], config, age, speed_cap=config.open_route_speed_mps)
            after=module.pursue_path(candidate, inp['speed_mps'], config, age, speed_cap=config.open_route_speed_mps)
            row.update(original_path_cap_mps=before[0], expanded_path_cap_mps=after[0],
                       original_steering_rad=before[1], expanded_steering_rad=after[1],
                       original_path_ahead_m=before[2], expanded_path_ahead_m=after[2],
                       near_arc_agrees=module.path_agrees_arc(path,candidate,config.boundary_agreement_m),
                       original_path=path.tolist(), expanded_path=candidate.tolist())
        except (ValueError, MotionUnavailable) as error:
            row['expanded_error']=str(error)
    records.append(row)
    return result


def events(source):
    global current, config
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':config=module.FollowConfig(**event['data']['follow_config'])
            current=event['data'] if event['kind']=='call' else None
            yield event


if __name__=='__main__':
    source=HERE.parent/'solo_corridor_arc_v1/run1/follow_decisions.jsonl'
    files=[Path(__file__).resolve(),*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    with ZipFile(HERE/'source.zip','x',ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,p.relative_to(ROOT).as_posix())
    before=sha256(source.read_bytes()).hexdigest()
    module.local_path=observe
    try:replay=replay_events(events(source))
    finally:module.local_path=original
    good=[r for r in records if 'expanded_path_cap_mps' in r]
    improved=[r for r in good if r['expanded_path_cap_mps']-r['original_path_cap_mps']>.05]
    unchanged=all(sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    summary=dict(calls=replay['calls'], matched=replay['matched'], inspected=len(records), expanded_paths=len(good),
        wall_growth_above_0p1m=sum((r.get('expanded_wall_ahead_m') or 0)-(r.get('original_wall_ahead_m') or 0)>.1 for r in records),
        path_cap_gain_above_0p05_mps=len(improved),
        improved_near_arc_disagreements=sum(not r['near_arc_agrees'] for r in improved),
        max_desired_steering_delta_rad=max((abs(r['expanded_steering_rad']-r['original_steering_rad']) for r in good),default=None),
        source_unchanged=unchanged,input_unchanged=before==sha256(source.read_bytes()).hexdigest())
    output=dict(scope='같은 현재 반환으로 벽 탐색 예산만 늘려 형상/경로 상한을 관찰. 실제 제어 출력은 불변이며 새 목표·주행·완주를 예측하지 않음.',
                summary=summary,replay={k:v for k,v in replay.items() if k!='outcomes'},source_sha256=hashes,
                input_sha256=before,rows=records)
    with (HERE/'probe.json').open('x',encoding='utf-8') as stream:json.dump(freeze(output),stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(freeze(summary),ensure_ascii=False))
    if not replay['passed'] or not unchanged or not summary['input_unchanged']:raise SystemExit(1)
