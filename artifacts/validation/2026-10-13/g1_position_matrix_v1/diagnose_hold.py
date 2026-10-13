"""첫 사후 감사 실패의 표본/밀집 자세/구동/판단 대조. 판정 변경 없음."""
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parent
RUN=BASE/'side_left_s1'


def read_rows(name):
    return [json.loads(line) for line in (RUN/name).read_text().splitlines() if line.strip()]


def main():
    out=BASE/'hold_diagnosis.json'
    if out.exists():
        raise ValueError('preserve earlier diagnosis')
    audit=json.loads((BASE/'side_left_s1.postcoverage.json').read_text())
    report=json.loads((RUN/'report.json').read_text())
    expected=dict(audit['input_sha256'])
    expected['braking_diagnostics.jsonl']=report['braking_diagnostics']['sha256']
    expected['follow_decisions.jsonl']=report['follow_decisions']['sha256']
    actual={name:hashlib.sha256((RUN/name).read_bytes()).hexdigest() for name in expected}
    if actual!=expected:
        raise ValueError('input differs from captured report/audit')
    pose=read_rows('ego_poses.jsonl')
    trace=json.loads((RUN/'trace.json').read_text())
    dynamics=read_rows('braking_diagnostics.jsonl')
    decisions=[row for row in read_rows('follow_decisions.jsonl') if row['kind']=='call']
    results=[]
    for interval in audit['hold_pose_contradictions']:
        start,end=interval['t0'],interval['t1']
        points=[row for row in pose if start<=row[0]<=end]
        if len(points)<2:
            raise ValueError('missing pose interval')
        segments=[dict(t0=a[0],t1=b[0],distance_m=math.hypot(b[1]-a[1],b[2]-a[2]),
            speed_mps=math.hypot(b[1]-a[1],b[2]-a[2])/(b[0]-a[0])) for a,b in zip(points,points[1:])]
        commands=[]
        for row in decisions:
            data=row['data']; t=data['input']['reference_s']
            if start<=t<=end:
                commands.append(dict(reference_s=t,measured_speed_mps=data['input']['speed_mps'],
                    requested_speed_mps=data['output'][0],reason=data['output'][2]['reason']))
        diag=[row for row in dynamics if start<=row['sim_time_s']<=end]
        if not diag or not commands:
            raise ValueError('missing command or dynamics interval')
        results.append(dict(interval=interval,pose_sample_count=len(points),
            covered_start_s=points[0][0],covered_end_s=points[-1][0],
            pose_max_gap_s=max(s['t1']-s['t0'] for s in segments),
            displacement_m=math.hypot(points[-1][1]-points[0][1],points[-1][2]-points[0][2]),
            traveled_m=sum(s['distance_m'] for s in segments),
            time_above_0p01_mps_s=sum(s['t1']-s['t0'] for s in segments if s['speed_mps']>.01),
            maximum_segment=max(segments,key=lambda s:s['speed_mps']),
            trace_endpoints=[dict(t=r['t'],ground_speed_mps=r['ground_speed_mps'],
                safe_speed_mps=r['safe_speed_mps'],reason=r['autonomy'].get('reason'))
                for r in trace if abs(r['t']-start)<1e-10 or abs(r['t']-end)<1e-10],
            decision_samples=commands,
            dynamics_samples=[{k:row[k] for k in ('sim_time_s','applied_target_mps',
                'motor_torque_nm','pi_integral_after_nm','truth_longitudinal_mps',
                'truth_lateral_mps','wheel_surface_speed_mps')} for row in diag],
            dense_truth_max_speed_mps=max(math.hypot(r['truth_longitudinal_mps'],r['truth_lateral_mps']) for r in diag),
            decision_zero_request=all(r['requested_speed_mps']==0 for r in commands),
            diagnostic_zero_target=all(r['applied_target_mps']==0 for r in diag)))
    result=dict(scope='failed_hold_window_diagnosis_not_verdict_change',windows=results,
        input_sha256=actual,inputs_unchanged=all(hashlib.sha256((RUN/name).read_bytes()).hexdigest()==value for name,value in actual.items()),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_postcoverage_passed=audit['passed'],original_run_passed=report['passed'],
        matrix_remaining_cases_authorized=False,limits=[
            'HOLD means zero speed request, not established physical stop.',
            'Sparse trace endpoints cannot certify every interior pose speed.',
            'Diagnostic torque and integral are sampled; correlation is not an isolated causal experiment.',
            'No physics, stop threshold, controller or original verdict was changed.'])
    out.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
