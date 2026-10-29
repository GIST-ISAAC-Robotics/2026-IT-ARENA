"""전환 실패의 명령·복귀 단계·실제 구동을 분리한다. 평가 자료만 읽는다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(run):
    trace = json.loads((run/'trace.json').read_text(encoding='utf-8'))
    moving = [r for r in trace if r.get('elapsed_since_green_s') is not None]
    green = moving[0]['t']-moving[0]['elapsed_since_green_s']
    rows = [json.loads(line) for line in (run/'g2_decisions.jsonl').open(encoding='utf-8')]
    calls = [r['data'] for r in rows if r['kind']=='call']
    phases, last = [], None
    for call in calls:
        value = call['output'][2]
        bypass = value.get('bypass', {})
        key = (value.get('handoff_state'), bypass.get('entry_phase'), bypass.get('reason'))
        if key != last:
            phases.append(dict(since_green_s=call['input']['reference_s']-green,
                handoff=key[0], entry_phase=key[1], reason=key[2],
                speed_command_mps=call['output'][0], steering_command_rad=call['output'][1]))
            last = key
    windows = {}
    for name, bounds in {'approach':(0.,4.8), 'transition':(4.8,6.), 'late':(6.,11.9)}.items():
        selected = [r for r in moving if bounds[0] <= r['elapsed_since_green_s'] <= bounds[1]]
        def stats(field, diagnostic=False):
            data = [r['braking_diagnostics'][field] if diagnostic else r[field] for r in selected]
            return dict(min=min(data), median=statistics.median(data), max=max(data)) if data else None
        windows[name] = dict(window_since_green_s=bounds, samples=len(selected),
            progress_delta_m=selected[-1]['progress_m']-selected[0]['progress_m'] if selected else None,
            drive=stats('drive_speed_mps'), safe=stats('safe_speed_mps'),
            safety_reasons=dict(Counter(r['safety_reason'] for r in selected)),
            truth_speed=stats('ground_speed_mps'),
            left_rear=stats('left_speed_rad_s',True),right_rear=stats('right_speed_rad_s',True),
            wheel_mean=stats('wheel_surface_speed_mps',True),
            motor_torque=stats('motor_torque_nm',True),
            roll=stats('truth_roll_rad',True),pitch=stats('truth_pitch_rad',True))
    return dict(run=str(run), input_sha256={name:sha(run/name) for name in
        ('trace.json','g2_decisions.jsonl','runtime_vehicle.sdf','runtime_world.sdf')},
        green_s=green, windows=windows, phase_transitions=phases,
        scope='Saved simulation evaluation only. Wheel-level diagnostics are not the motor encoder input. Contact cause requires separate evidence.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze(args.run)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(phase_transitions=result['phase_transitions'],late=result['windows']['late']),indent=2))
