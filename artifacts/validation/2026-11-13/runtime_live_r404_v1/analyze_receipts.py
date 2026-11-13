"""저장된 r404 수신 자료만 집계한다. 실제 장치/ROS를 다시 실행하지 않는다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


def summarize(values):
    values = sorted(values)
    assert values and all(math.isfinite(value) for value in values)
    return dict(n=len(values), minimum=min(values), median=statistics.median(values),
                p95=values[math.ceil(len(values)*.95)-1], maximum=max(values))


def main():
    here = Path(__file__).resolve().parent
    source = here/'remote_result/extracted/actual/node'
    hashes = {}

    def rows(name):
        path = source/(name+'.json')
        raw = path.read_bytes()
        hashes[name] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)

    scans, images, gyro = rows('scans'), rows('images'), rows('motion')
    vision, commands = rows('vision'), rows('commands')
    result = dict(
        scan_first_estimated_age_ms=summarize([(r['received_s']-r['first_s'])*1000 for r in scans]),
        scan_last_estimated_age_ms=summarize([(r['received_s']-r['last_s'])*1000 for r in scans]),
        camera_host_receipt_to_node_ms=summarize([(r['received_ns']-r['stamp_ns'])/1e6 for r in images]),
        gyro_estimated_age_ms=summarize([(r['received_s']-r['stamp_s'])*1000 for r in gyro]),
        vision_processing_ms=summarize([r['processing_ms'] for r in vision]),
        vision_completion_wait_ms=summarize([r['completion_wait_ms'] for r in vision]),
        states=dict(Counter(r['state'] for r in rows('states'))),
        guard_reasons=dict(Counter(r['reason'] for r in rows('guard_states'))))
    for stage in ('before_guard', 'after_guard'):
        selected = [r for r in commands if r['stage'] == stage]
        assert all(r['speed'] == r['steering'] == 0 for r in selected)
        result[stage+'_command_age_ms'] = summarize([(r['received_ns']-r['stamp_ns'])/1e6 for r in selected])
        result[stage+'_header_interval_ms'] = summarize([
            (v['stamp_ns']-u['stamp_ns'])/1e6 for u, v in zip(selected, selected[1:])])
    result.update(input_sha256=hashes, source_time_verified=False, encoder_present=False,
        full_planning_throughput_measured=False,
        scope='saved zero-output receipt analysis; scan/gyro times estimated; camera age begins at host receipt, not exposure')
    target = here/'receipt_analysis.json'
    with target.open('x', encoding='utf8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
