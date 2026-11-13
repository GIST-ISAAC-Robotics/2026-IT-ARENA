"""r364 원 현재 검사 재현과 동일 조향의 저속 영역 진단. 명령 발행 없음."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path.insert(0, str(A.parent/'runtime_factory_r365_v1'))
from factory_reference import evaluator_factory
from arena_autonomy.follow_trace import restore_call, thaw, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.runtime_dual_process import plan_reference
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.released_body_gap import check
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn


def main():
    path = A.parent/'released_crawl_drive_r364_v1/run1/follow_decisions.jsonl'
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest == '1aab37deed9f1da83792988c1b98d7239b0ecb87a5335ba4a8ecc5b0fc7e3821'
    events = [json.loads(line) for line in path.open()]
    header = dict(events[0]['data'], r365_policy='r353', numeric_execution='native_v1')
    evaluator = evaluator_factory('current', header)
    cfg = evaluator.follow.config
    rows = [e['data'] for e in events if e['kind']=='call' and 'current_rebase' in e['data']]
    checks, samples = [], []
    selected = set(range(max(0, len(rows)-160), len(rows), 10)) | {len(rows)-1}
    for index, row in enumerate(rows):
        c = row['current_rebase']
        plan = c['compute']['plan']
        current = c['input']
        data = dict(output=plan['output'], submitted=plan_reference(plan['input']), current=current)
        actual = evaluator(data)
        expected = c['output']
        # ROS 부모에서 붙인 같은 양수 감속 상한만 원 기록대로 적용한다.
        cap = expected.get('new_plan_deceleration_cap')
        if cap is not None:
            assert equal_output(actual['speed_mps'], cap['checked_target_mps'])
            actual = {**actual, 'speed_mps': min(actual['speed_mps'], cap['speed_mps']),
                      'new_plan_deceleration_cap': cap}
        same = equal_output(actual, expected)
        checks.append(dict(index=index, reference_s=current['reference_s'], equal=same,
                           reason=actual['reason']))
        if not same:
            (A/'mismatch.json').write_text(json.dumps(dict(index=index,actual=actual,expected=expected)))
            raise AssertionError((index, 'original current mismatch'))
        if index not in selected:
            continue
        info, scan, history = restore_call(current, evaluator.motion)
        scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                             epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
        raw = MeasurementScan(**thaw(current['scan']['measurements']))
        attempts = []
        for target in (.15, .10, .05):
            core = evaluator.current_policy.prior.prior.base.candidate_policy.rebase(
                (target, plan['output'][1], plan['output'][2]), data['submitted'], current, scene, history, cfg)
            if core is None or core['speed_mps'] <= 0:
                attempts.append(dict(target=target, core_stop=None if core is None else core['reason']))
                continue
            context = {}
            ps.motion_region(scene, current['speed_mps'], core['speed_mps'], core['steering_rad'],
                             current['previous_steering_rad'], current['dt_s'], cfg, ps.lateral_config(cfg), context=context)
            q = context.get('queries', np.empty((0,2)))
            unknown = q[~context['free'] & ~context['masked']] if len(q) else q
            checked = check(scene, raw, current['speed_mps'], core['speed_mps'], core['steering_rad'],
                current['previous_steering_rad'], current['dt_s'], cfg, ps.lateral_config(cfg), ps.SelectorConfig())
            relative = None
            if len(unknown) and checked['proof'].get('reason')=='near_gap_neighbor':
                # 진단만: 현재 필요한 모든 질의와 모든 광선 padding을 포함한 보수적 반경.
                # 값이 작다는 이유로 정책을 채택하지 않는다. 아래 결과로 실제 이웃을 확인한다.
                origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
                reach = float(np.max(np.linalg.norm(unknown[:,None,:]-origins[None,:,:],axis=2)))
                minimum = reach + float(np.max(context['padding']))
                relative = _bounded_nonreturn(scene, raw, unknown, context['padding'],
                    {**checked['proof'], 'minimum_neighbor_range_m':minimum})
                relative['query_reach_m'] = reach
                relative['max_padding_m'] = float(np.max(context['padding']))
            attempts.append(dict(target=target, check=checked, steering=core['steering_rad'],
                relative_neighbor_diagnostic=relative,
                unknown_bbox_m=None if not len(unknown) else [unknown.min(axis=0).tolist(),unknown.max(axis=0).tolist()],
                outside_longitudinal_count=int(np.sum(np.abs(unknown[:,0])>cfg.length_m/2)),
                side_exterior_max_m=None if not len(unknown) else float(np.max(np.abs(unknown[:,1])-cfg.width_m/2))))
        samples.append(dict(index=index, reference_s=current['reference_s'], measured_speed=current['speed_mps'],
            mode=plan['output'][2].get('normal',{}).get('selector_mode'), attempts=attempts))
    report = dict(trace_sha256=digest, original_current_count=len(checks), original_equal=sum(r['equal'] for r in checks),
        geometry=dict(length_m=cfg.length_m,width_m=cfg.width_m,wheelbase_m=cfg.wheelbase_m), samples=samples,
        scope='saved current inputs, no new driving, steering or thresholds unchanged')
    (A/'probe_relative.json').write_text(json.dumps(freeze(report),indent=2)+'\n')
    print(json.dumps(dict(original_current_count=len(checks),original_equal=report['original_equal'],
        selected=len(samples),accepted_by_target={str(t):sum(a.get('check',{}).get('accepted',False) for r in samples for a in r['attempts'] if a['target']==t) for t in (.15,.10,.05)},last=samples[-1])))
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest


if __name__=='__main__':main()
