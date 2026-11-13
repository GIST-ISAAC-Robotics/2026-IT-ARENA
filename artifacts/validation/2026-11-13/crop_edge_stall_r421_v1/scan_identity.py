"""원 scan 배열/metadata만 canonical SHA로 대조. history/허가 동등성을 주장하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
SOURCE = A.parent/'crop_edge_drive_r418_v1'
FIELDS = ('ranges_m', 'angles_rad', 'acquired_at_s', 'source_frame_id', 'source_epoch',
          'time_basis', 'range_min_m', 'range_max_m')


def sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                   allow_nan=False).encode()).hexdigest()


def main():
    base = json.loads((A/'summary.json').read_bytes())
    start, end = (base['last_10_sim_seconds'][key] for key in ('start_s','end_s'))
    all_counts, late_counts = Counter(), Counter()
    plans = {}; examples = []; late_examples = []
    for line in (SOURCE/'run1/follow_decisions.jsonl').open(encoding='utf8'):
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        data = event['data']; compute = data['compute']; output = data['output']
        pid = (compute['generation'], compute['sequence'])
        partial = output[2].get('normal', output[2]).get('partial_observation') or {}
        plans.setdefault(pid, dict(speed=output[0], crop='crop_edge_component' in partial))
        rebase = data.get('current_rebase')
        if rebase is None:
            continue
        scan = rebase['input']['scan']; original = data['input']['scan']
        submitted = rebase['compute']['plan']['input']['scan']
        reference = data['input']['reference_s']
        late = start <= reference <= end
        hashes = dict(outer_plan=sha(original), submitted_plan=sha(submitted), current=sha(scan))
        same = len(set(hashes.values())) == 1
        for counts in (all_counts, late_counts) if late else (all_counts,):
            counts['current_records'] += 1
            counts['exact_three_way_scan_sha'] += same
            for name in FIELDS:
                counts[name+'_equal'] += (sha(scan['measurements'][name]) ==
                    sha(original['measurements'][name]) == sha(submitted['measurements'][name]))
        if late or event['seq'] in (680,689,713):
            native = scan['measurements']
            item = dict(trace_seq=event['seq'], plan_id=pid,
                current_id=[rebase['compute']['generation'], rebase['compute']['sequence']],
                plan_reference_s=reference, current_reference_s=rebase['input']['reference_s'],
                three_way_sha256=hashes, fields_equal={name: sha(native[name]) == sha(original['measurements'][name])
                    == sha(submitted['measurements'][name]) for name in FIELDS},
                measurement_count=len(native['ranges_m']), source_frame_id=native['source_frame_id'],
                source_epoch=native['source_epoch'], time_basis=native['time_basis'],
                acquired_first_s=native['acquired_at_s'][0], acquired_last_s=native['acquired_at_s'][-1])
            if event['seq'] in (680,689,713):
                examples.append(item)
            if late:
                late_examples.append(item)
    report = dict(scope='exact stored native scan content only; not whole history/current proof/permission reuse',
        comparison='SHA256 of canonical JSON with sorted object keys; array order/float values/nonfinite tags unchanged',
        input_sha256=base['input_sha256'], last_window_s=[start,end],
        unique_plans=len(plans), unique_positive_plans=sum(p['speed']>0 for p in plans.values()),
        unique_crop_plans=sum(p['crop'] for p in plans.values()),
        unique_positive_crop_plans=sum(p['crop'] and p['speed']>0 for p in plans.values()),
        all_current_records=dict(all_counts), last10=dict(late_counts),
        representatives=[*examples,late_examples[0],late_examples[-1]])
    with (A/'scan_identity.json').open('x', encoding='utf8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False);stream.write('\n')
    print(json.dumps(dict(all_current=all_counts,last10=late_counts,
        unique_plans=report['unique_plans'],unique_positive_plans=report['unique_positive_plans'],
        unique_crop_plans=report['unique_crop_plans'],unique_positive_crop_plans=report['unique_positive_crop_plans']),
        ensure_ascii=False))


if __name__ == '__main__':
    main()
