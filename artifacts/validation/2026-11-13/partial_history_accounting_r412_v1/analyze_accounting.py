"""r410 저장 출력/상태와 원 scan 시각만 읽는다. 정책·numpy·ROS 재생은 하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PRIOR = A.parent/'front_crop_boundary_r410_v1'
INPUTS = {
    'baseline': PRIOR/'verify_native_v1/baseline.jsonl',
    'candidate': PRIOR/'verify_native_v1/candidate.jsonl',
    'comparison_report': PRIOR/'verify_native_v1/report.json',
    'geometry': PRIOR/'analysis_v1/r406_rows.jsonl',
    'trace': A.parent/'final_runtime_drive_r406_v1/run1/follow_decisions.jsonl',
    'partial_source': ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py',
    'timed_source': ROOT/'src/arena_autonomy/arena_autonomy/traffic_recovery_trial.py',
    'structure_source': ROOT/'src/arena_autonomy/arena_autonomy/known_structure.py',
    'geometry_source': ROOT/'src/arena_autonomy/arena_autonomy/recovery_geometry.py',
    'r410_manifest': PRIOR/'manifest.json',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.open(encoding='utf8')]


def details(row):
    result = row['output'][2]
    return result.get('normal', result)


def snapshot(row):
    state = row['state']; previous = state['previous']; distinct = state['previous_distinct']
    partial = details(row).get('partial_observation', {})
    return dict(trace_seq=row['trace_seq'], plan_sequence=row['sequence'], reference_s=row['reference_s'],
        command_speed_mps=row['output'][0], reason=details(row).get('reason'), partial_reason=partial.get('reason'),
        previous=None if previous is None else dict(points=len(previous[0]), reference_s=previous[1], source_s=previous[2]),
        previous_distinct=None if distinct is None else dict(points=len(distinct[0]), reference_s=distinct[1], source_s=distinct[2]),
        last_source=state['last_source'], clear_sources=state['clear_sources'],
        latched=state['latched'], latch_reason=state['latch_reason'], latch_time=state['latch_time'],
        latch_evidence_present=partial.get('latch_evidence') is not None,
        timed_missing_diagnostic_present='timed_missing_diagnostic' in partial)


def input_summary(data):
    scan = data['scan']['measurements']
    values = [float(v['nonfinite']) if isinstance(v, dict) else v for v in scan['ranges_m']]
    valid = [math.isfinite(v) and scan['range_min_m'] <= v <= scan['range_max_m'] for v in values]
    motion = {}
    for name in ('wheels', 'gyro'):
        times = data['history'][name]['times']
        motion[name] = dict(samples=len(times), first_s=times[0], last_s=times[-1],
            maximum_input_gap_s=max(b-a for a,b in zip(times,times[1:])),
            reference_extrapolation_s=data['reference_s']-times[-1])
    return dict(source_first_s=scan['acquired_at_s'][0], source_last_s=scan['acquired_at_s'][-1],
        source_age_s=data['reference_s']-scan['acquired_at_s'][0],
        rays=len(values), valid_returns=sum(valid), source_frame_id=scan['source_frame_id'],
        source_epoch=scan['source_epoch'], time_basis=scan['time_basis'], timing_verified=data['timing_verified'],
        measured_speed_mps=data['speed_mps'], previous_steering_rad=data['previous_steering_rad'], motion_history=motion)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    destination=args.output.resolve();assert destination.is_relative_to(A) and not destination.exists()
    hashes={key:digest(path) for key,path in INPUTS.items()}
    candidate=rows(INPUTS['candidate']);baseline={r['trace_seq']:r for r in rows(INPUTS['baseline'])}
    first=next(i for i,r in enumerate(candidate) if details(r).get('reason')=='follow_partial_history_expired')
    previous=candidate[first]['state']['previous']
    remembered=max(i for i in range(first) if candidate[i]['state']['previous']==previous
        and (i==0 or candidate[i-1]['state']['previous']!=previous))
    selected=candidate[remembered:first+1]
    sequences={r['trace_seq'] for r in selected}
    raw={}
    for line in INPUTS['trace'].open(encoding='utf8'):
        event=json.loads(line)
        if event['kind']=='header':header=event['data']
        if event['kind']=='call' and event['seq'] in sequences:raw[event['seq']]=event['data']['input']
    geometry={r['trace_seq']:r['edges'] for r in rows(INPUTS['geometry']) if r['trace_seq'] in sequences}
    timeline=[]
    for r in selected:
        timeline.append(dict(candidate=snapshot(r), baseline=snapshot(baseline[r['trace_seq']]),
            source=input_summary(raw[r['trace_seq']]), edges=geometry[r['trace_seq']]))
    transitions=[];old=None
    for r in candidate:
        state=r['state'];key=(state['latched'],state['latch_reason'],state['latch_time'])
        if key!=old:transitions.append(snapshot(r));old=key
    assert [r['trace_seq'] for r in selected]==[684,685,686,687,688]
    assert all(r['source']['source_age_s'] < .3 for r in timeline)
    assert all(r['candidate']['partial_reason']=='unbounded_missing' for r in timeline[1:-1])
    assert all(r['candidate']['previous']==timeline[0]['candidate']['previous'] for r in timeline)
    report=dict(scope='Saved r410 limited replay diagnosis, not original live child-state reconstruction or new policy execution',
        input_sha256={INPUTS[k].relative_to(ROOT).as_posix():v for k,v in hashes.items()},
        first_expiry=dict(index=first, trace_seq=candidate[first]['trace_seq'], plan_sequence=candidate[first]['sequence'],
            reference_s=candidate[first]['reference_s'], last_remember_trace_seq=candidate[remembered]['trace_seq'],
            previous_reference_s=previous[1], previous_source_s=previous[2], previous_points=len(previous[0]),
            previous_age_s=candidate[first]['reference_s']-previous[1]),
        intervening_early_returns=dict(Counter(details(r).get('reason') for r in selected[1:-1])),
        source_intervals_s=[b['source']['source_first_s']-a['source']['source_first_s'] for a,b in zip(timeline,timeline[1:])],
        motion_limits={k:header['motion_config'][k] for k in ('max_input_gap_s','max_extrapolation_s')},
        timeline=timeline, candidate_latch_transitions=transitions,
        interpretation='Current valid returns and source acquisition times advance, but geometry rejection bypasses both threat assessment and _remember. No saved threat assessment proves this interval harmless.')
    assert all(digest(INPUTS[k])==v for k,v in hashes.items())
    with destination.open('x',encoding='utf8') as stream:json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({k:report[k] for k in ('first_expiry','intervening_early_returns','source_intervals_s')},ensure_ascii=False))


if __name__=='__main__':main()
