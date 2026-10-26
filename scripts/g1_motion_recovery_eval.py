"""G1 교통 기록의 IMU 지연 burst를 기존 정지·복구 기준으로 사후 평가한다.

평가용 실제 자세만 읽으며 제어기로 전달하지 않는다. 원래 보고서는 수정하지 않고,
기존 9/22 dynamic_audit 기준에 밀집 자세 연속성을 추가한다.
"""
import argparse
import hashlib
import json
from pathlib import Path

from g1_traffic_eval import kinematic_speed_segments, pose_window_coverage
from summarize_motion_robustness import dynamic_audit


def evaluate(trace, ego, config):
    fault=config.get('imu',{})
    if fault.get('delay_s') != .08 or fault.get('start_s') != 8. or fault.get('end_s') != 10.:
        raise ValueError('registered IMU 80 ms burst at 8..10 s required')
    if fault.get('drop',False) or any(config.get('wheels',{}).get(k,default)!=default
            for k,default in (('scale',1.),('bias',0.),('delay_s',0.),('jitter_s',0.),('stamp_offset_s',0.),('drop',False))):
        raise ValueError('no motor impairment or IMU dropout in this registered condition')
    if not trace or any(b['t']<=a['t'] for a,b in zip(trace,trace[1:])):
        raise ValueError('nonempty increasing trace required')
    converted=[dict(time=r['t'],x=r['x'],y=r['y'],progress_m=r['progress_m'],
                    dynamics={'truth_longitudinal_mps':r['ground_speed_mps'],'truth_lateral_mps':0.},
                    safe_drive_speed_mps=r['safe_speed_mps'],safety={'reason':r['safety_reason']}) for r in trace]
    old=dynamic_audit(converted,config)
    segments=kinematic_speed_segments(ego)
    dense={'complete':False,'passed':False,'reason':'no_stable_window'}
    if old['stable_stop_during_fault']:
        start=8.+old['stop_after_fault_s']
        dense=pose_window_coverage(segments,start,start+.3)
    # 평가 trace의 긴 구멍을 연결해 명령 0 또는 복구 증거로 만들지 않는다.
    relevant=[r for r in trace if 7.5<=r['t']<=10.5]
    trace_continuous=bool(relevant and relevant[0]['t']<=7.7 and relevant[-1]['t']>=10.3
                          and all(b['t']-a['t']<=.20+1e-9 for a,b in zip(relevant,relevant[1:])))
    return dict(passed=bool(old['passed'] and dense['passed'] and trace_continuous),
                sampled_stop_recovery=old,dense_stop=dense,trace_continuous=trace_continuous,
                limits=['One registered burst, not all delays or physical braking.',
                        'Status response is sampled at about 10 Hz; pose checks use recorded segments.',
                        'Original traffic, motion injection and shutdown verdicts remain required.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    files={n:(args.run/n).read_bytes() for n in ('report.json','trace.json','ego_poses.jsonl')}
    report=json.loads(files['report.json'])
    result=evaluate(json.loads(files['trace.json']),[json.loads(x) for x in files['ego_poses.jsonl'].splitlines()],
                    report['motion_test_profile']['requested'])
    result.update(original_passed=report['passed'],original_harness_valid=report['harness_valid'],
                  motion_injection_verified=report.get('motion_stress_evidence',{}).get('verified') is True,
                  input_sha256={n:hashlib.sha256(b).hexdigest() for n,b in files.items()},
                  input_unchanged=all((args.run/n).read_bytes()==b for n,b in files.items()))
    result['combined_passed']=bool(result['passed'] and result['original_passed'] and result['original_harness_valid']
                                   and result['motion_injection_verified'] and result['input_unchanged'])
    with args.output.open('x',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['combined_passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
