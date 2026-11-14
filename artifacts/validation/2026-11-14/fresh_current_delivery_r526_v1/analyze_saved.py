"""r523 원시 저장 시각만 분석. 새 실행/인과 반사실/TTL 통과를 합성하지 않는다."""
from pathlib import Path
import gzip
import hashlib
import json
import math

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
SOURCE = A.parent/'jetson_drain_budget_r523_v1/remote_result/result'


def quantiles(values):
    values = sorted(values)
    if not values:
        return dict(n=0)
    def at(q):
        p = q*(len(values)-1)
        i = int(p)
        return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(p-i)
    return dict(n=len(values),minimum=values[0],median=at(.5),p95=at(.95),maximum=values[-1])


inputs = ['commands.jsonl.gz','controller_controls.json','calculation_receipts.json']
payloads = {name:(SOURCE/name).read_bytes() for name in inputs}
commands = [json.loads(line) for line in gzip.decompress(payloads['commands.jsonl.gz']).splitlines()]
commands = [row for row in commands if row['topic']=='/drive' and 7.62<=row['received_s']<27.52]
controls = json.loads(payloads['controller_controls.json'])
receipts = json.loads(payloads['calculation_receipts.json'])
positive = [row for row in commands if row['speed']>0]
by_source = {round(row['source_s']*1e9):row for row in positive}
previous = {id(row):commands[index-1] for index,row in enumerate(commands) if index>0}
matches=[];unmatched_submission=0;unmatched_delivery=0
for receipt in receipts:
    if receipt['stage'] != 'current':
        continue
    stamp=receipt['submitted_wall_s']*1e9
    owners=[row for row in controls if row['begin_monotonic_ns']<=stamp<=row['end_monotonic_ns']]
    if len(owners)!=1:
        unmatched_submission+=1
        continue
    control=owners[0]
    ref=control['control_reference_s']
    command=by_source.get(round(ref*1e9))
    if command is None:
        unmatched_delivery+=1
        continue
    prev=previous.get(id(command))
    matches.append(dict(sequence=receipt['sequence'],generation=receipt['generation'],
        original_current_reference_s=ref,
        current_received_wall_s=receipt['received_wall_s'],command_received_wall_s=command['received_wall_s'],
        current_collection_to_command_observer_ms=1000*(command['received_wall_s']-receipt['received_wall_s']),
        command_age_at_observer_ms=1000*(command['received_s']-command['source_s']),
        remaining_command_ttl_at_observer_ms=1000*(.2-command['received_s']+command['source_s']),
        previous_command=None if prev is None else dict(source_s=prev['source_s'],
            received_wall_s=prev['received_wall_s'],speed=prev['speed'],steering=prev['steering']),
        received_spacing_ms=None if prev is None else 1000*(command['received_wall_s']-prev['received_wall_s']),
        ready_receipt_after_previous_observer_ms=None if prev is None else
            1000*(receipt['received_wall_s']-prev['received_wall_s']),
        steering_change_rate_budget_ms=None if prev is None else
            1000*abs(command['steering']-prev['steering'])/3.,
        delivered_steering=command['steering'],source_stamp_unchanged=True))

angle_intervals=[1000*abs(row['steering']-previous[id(row)]['steering'])/3.
                 for row in positive if id(row) in previous]
report=dict(scope='saved input timing analysis; observer receipt is not publication/guard callback time',
    source_sha256={name:hashlib.sha256(value).hexdigest() for name,value in payloads.items()},
    positive_commands=len(positive),unique_positive_steering=len(set(row['steering'] for row in positive)),
    exact_equal_positive_consecutive_angles=sum(a['steering']==b['steering'] for a,b in zip(positive,positive[1:])),
    steering_change_required_interval_ms=quantiles(angle_intervals),
    angle_change_fits_20ms=sum(v<=20. for v in angle_intervals),
    source_age_at_observer_ms=quantiles(1000*(r['received_s']-r['source_s']) for r in positive),
    remaining_command_ttl_at_observer_ms=quantiles(1000*(.2-r['received_s']+r['source_s']) for r in positive),
    current_receipts=sum(r['stage']=='current' for r in receipts),matched_current_delivery=len(matches),
    unmatched_submission=unmatched_submission,unmatched_delivery=unmatched_delivery,
    current_collection_to_command_observer_ms=quantiles(r['current_collection_to_command_observer_ms'] for r in matches),
    matched_ready_before_100ms_after_previous_observer=sum(
        r['ready_receipt_after_previous_observer_ms'] is not None and
        0<=r['ready_receipt_after_previous_observer_ms']<100 for r in matches),
    boundaries=[
        'Mapping uses current submission within one measured control callback and equal delivered source stamp.',
        'Collection-to-observer includes subsequent callback work and subscriber scheduling, not only the 100ms gate.',
        'Trace OFF omitted full ready output, output dt and withheld-candidate transitions; exact policy replay is unavailable.',
        'Observed float32 steering differences are timing-budget diagnostics, not substituted checked float64 commands.',
        'Early publication does not extend the absolute source_reference+200ms command expiry.',
        'No estimate here proves 17.95% zero-target time reduction or a physical driving improvement.',
    ],matches=matches)
(A/'saved_timing.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('matches','source_sha256','boundaries')}))
assert all((SOURCE/name).read_bytes()==value for name,value in payloads.items())
