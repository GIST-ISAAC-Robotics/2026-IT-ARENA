"""합성 곡선로에서 해제되지 않은 M3의 '정지 중 재출발'과 '이동 중 통과' 비교 탐침. 개루프 단일 판단 진단.

반경 R의 왼쪽 곡선 벽(짧은 상자 연결, 도로 반폭 0.425 m)을 자차 좌표에서 광선 투사한다. 이동 사례는 일정 속도·
요레이트로 곡선 위를 지나온 이전 두 장면을 같은 운동 이력과 함께 SceneHistory에 넣는다. 정지 사례는 같은 자세의
이전 장면이다. 측방 시험은 활성·만료 위협 기록(복귀 불가)으로 주입하고, 후보 검사는 실제 계획·명령 검사를 쓴다.
스캔 중 자차 이동은 광선별로 넣지 않았다(시험 fixture와 같은 정지 투사). 주행 결과가 아니다.
"""
import json
import math
from pathlib import Path
import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.occupancy_timing import per_ray_padding
from test_lateral_intent import box, cast
from test_local_follow import history
from test_m3_execution_trial import ctl


def wall_boxes(radius, offset, thickness=.05, start=-80., stop=200., step=2.):
    """곡선 중심 (0, R)의 원호 벽. offset>0은 바깥(오른쪽), <0은 안쪽(왼쪽)."""
    boxes = []
    for a0 in np.arange(start, stop, step):
        a = np.radians([a0, a0+step])
        r_in, r_out = radius+offset, radius+offset+math.copysign(thickness, offset)
        pts = [(r*math.sin(t), radius-r*math.cos(t)) for r, t in ((r_in, a[0]), (r_in, a[1]), (r_out, a[1]), (r_out, a[0]))]
        boxes.append(np.array(pts))
    return boxes


def pose_on_arc(radius, back_m):
    th = -back_m/radius
    return radius*math.sin(th), radius*(1-math.cos(th)), th


def scene_from_pose(now, radius, pose):
    x, y, yaw = pose
    c, s = math.cos(-yaw), math.sin(-yaw)
    rot = np.array([[c, -s], [s, c]])
    polys = [(p-[x, y])@rot.T for p in wall_boxes(radius, .425)+wall_boxes(radius, -.425)]
    sc = cast(polys)
    delta = now-sc.t_ref_s
    return replace(sc, t_ref_s=now, source_first_s=sc.source_first_s+delta, source_last_s=sc.source_last_s+delta,
                   rays=tuple(replace(r, acquired_at_s=r.acquired_at_s+delta) for r in sc.rays))


def path_points(radius):
    s = np.linspace(-.3, 1.6, 80)
    return np.column_stack((radius*np.sin(s/radius), radius*(1-np.cos(s/radius))))


def stale():
    return mx.ThreatRecord('left', SimpleNamespace(t_ref_s=0., source_epoch=-1, source_first_s=0.),
                           np.zeros((1, 2)), np.zeros(1), None, 0.)


def probe(radius, moving, previous, small):
    execution = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True, small_center_straight=small)
    follow = ctl(execution)
    v = .34 if moving else 0.
    h = history(v=v, w=v/radius)
    cache = mx.SceneHistory()
    frames = []
    for k, now in enumerate((.8, .9, 1.0)):
        back = v*(1.0-now)
        sc = scene_from_pose(now, radius, pose_on_arc(radius, back))
        frames = cache.update(sc, h)
    trial = follow.lateral
    trial.active, trial.clear_fresh, trial.records = True, 2, {'left': stale()}
    trial.epoch, trial.last_ref, trial.last_source = sc.source_epoch, .9, sc.source_first_s-.1
    path = path_points(radius)
    padding = per_ray_padding(sc, .025+follow.config.measurement_padding_m, follow.config.object_motion_bound_mps)
    baseline = (0., 0., {'path_points': path.tolist(), 'boundary_age_s': .1})
    seen = {}

    def check(name, candidate_path):
        result = follow._evaluate_command(name, candidate_path, sc, frames, h, v, previous, .02, padding, baseline)
        env = result['detail'].get('m3_envelope') or {}
        seen[name] = {'accepted': result['accepted'], 'reason': result['reason'], 'steering': result.get('steering'),
                      'unsupported': (env.get('observation') or {}).get('unsupported'),
                      'masked': (env.get('observation') or {}).get('masked_candidates'),
                      'history_supported': (env.get('observation') or {}).get('history_supported')}
        return {'accepted': result['accepted'], 'reason': result['reason']}

    out = trial.step(sc, frames, path, max(v, follow.trial_speed_cap_mps), padding, check, history=h)
    return {'radius_m': radius, 'moving': moving, 'previous_steering_rad': previous, 'small_center_straight': small,
            'history_frames': len(frames), 'mode': out['mode'], 'selected': out['selected'],
            'small_center': out.get('small_center'),
            'plan': {k: {'reasons': c['reasons'], 'obs': (c.get('observation') or {}).get('unsupported')}
                     for k, c in out['checks'].items()},
            'command': seen}


def main():
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    cases = [probe(r, m, p, s) for r in (1.2, 2.0) for m in (False, True) for p in (0., .1) for s in (False, True)]
    output.write_text(json.dumps({'scope': 'synthetic arc road, injected unreleased M3; single-decision open-loop probe',
                                  'cases': cases}, ensure_ascii=False, indent=1)+'\n', encoding='utf8')
    for c in cases:
        print(json.dumps(c, ensure_ascii=False))


if __name__ == '__main__':
    main()
