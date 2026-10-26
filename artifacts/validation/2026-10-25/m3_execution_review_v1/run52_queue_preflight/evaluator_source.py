"""M2 두 앞차 대기열의 평가 전용 검사. 정답은 제어기로 전달하지 않는다."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import g1_traffic_eval as ev


def queue_window(trace, ego, near, far, first, last, minimum_far_travel):
    """먼 차가 출발하고 가까운 차는 멈춘 구간의 실제 자극과 자차 대기를 분리한다."""
    ego_hold = ev.pose_window_coverage(ev.kinematic_speed_segments(ego), first, last)
    near_hold = ev.pose_window_coverage(ev.kinematic_speed_segments(near), first, last)
    far_segments = ev.kinematic_speed_segments(far)
    far_cover = ev.pose_window_coverage(far_segments, first, last)
    # 구간 내부 표본만 이용하므로 창 밖 움직임을 이동량에 보태지 않는다.
    inside = [p for p in far if first <= p[0] <= last]
    travel = math.hypot(inside[-1][1]-inside[0][1], inside[-1][2]-inside[0][2]) if len(inside)>1 else 0.
    times = [r['t'] for r in trace]
    trace_valid = bool(times) and all(math.isfinite(t) for t in times) and all(b>a for a,b in zip(times,times[1:]))
    # 창 양 끝을 포함하는 직전/직후 표본까지 요구한다. 누락된 구간을 HOLD로 메우지 않는다.
    selected = []
    if trace_valid:
        before = [i for i,t in enumerate(times) if t <= first]
        after = [i for i,t in enumerate(times) if t >= last]
        if before and after:
            selected = trace[before[-1]:after[0]+1]
    trace_covered = bool(selected) and all(0 < b['t']-a['t'] <= ev.TRACE_MAX_GAP_S+1e-9 for a,b in zip(selected,selected[1:]))
    held = bool(selected) and all(r.get('safe_speed_mps') == 0. and r.get('autonomy',{}).get('behavior') == 'HOLD' for r in selected)
    gates = {'ego_stationary':ego_hold['passed'], 'near_stationary':near_hold['passed'],
             'far_pose_covered':far_cover['complete'], 'far_moved':travel >= minimum_far_travel,
             'trace_covered':trace_covered, 'safe_zero_and_hold':held}
    return {'passed':all(gates.values()), 'gates':gates, 'window_s':[first,last],
            'ego_pose':ego_hold, 'near_pose':near_hold, 'far_displacement_m':travel,
            'far_pose_coverage':far_cover['complete'], 'trace_samples':len(selected)}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    names=('report.json','trace.json','ego_poses.jsonl','actor_poses.jsonl','additional_actors.json')
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    before={n:digest(args.run/n) for n in names}
    cfg=json.loads(args.config.read_text(encoding='utf8'))
    report=json.loads((args.run/'report.json').read_text(encoding='utf8'))
    if report['case'] != cfg['base_case'] or report['actor_overlay']['sha256'] != digest(args.config):
        raise ValueError('run and preregistered overlay differ')
    load_rows=lambda name:[json.loads(l) for l in (args.run/name).read_text(encoding='utf8').splitlines()]
    extra=json.loads((args.run/'additional_actors.json').read_text(encoding='utf8'))
    trace=json.loads((args.run/'trace.json').read_text(encoding='utf8'))
    green=report['evaluation']['t_green']
    rules=cfg['queue_acceptance']
    first,last=[green+t for t in rules['far_moves_near_stopped_window_s']]
    result=queue_window(trace,load_rows('ego_poses.jsonl'),load_rows('actor_poses.jsonl'),
        extra['poses']['eval_m2_far_lead'],first,last,rules['minimum_far_travel_m'])
    out={'scope':'evaluation only; not hidden-car tracking or continuous safety proof',
         'original_passed':report['passed'], 'original_run_valid':report['run_valid'],
         'queue':result, 'registered_combined_passed':bool(report['passed'] and result['passed']),
         'input_sha256':before,'config_sha256':digest(args.config),
         'inputs_unchanged':all(digest(args.run/n)==v for n,v in before.items())}
    assert out['inputs_unchanged']
    args.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))


if __name__=='__main__':
    main()
