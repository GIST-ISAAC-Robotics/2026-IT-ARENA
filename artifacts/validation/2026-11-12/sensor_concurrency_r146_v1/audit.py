"""원시 기록을 대조한다. 센서 동기화/실차 제어 판정기는 아니다."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import statistics

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = ROOT/'build/sensor_concurrency_r146/remote/it_arena_sensor_concurrency_r146_20261112_v2'
C = BASE/'capture'


def stats(values):
    a = sorted(values)
    return dict(count=len(a), minimum=min(a), median=statistics.median(a),
                p95=a[int(.95*(len(a)-1))], maximum=max(a)) if a else dict(count=0)


def main():
    paths = [C/'c1-scans.csv', C/'c1-points.csv', C/'imu/frames.jsonl', C/'imu/uart.bin',
             C/'imu/summary.json', C/'camera/sent.csv', C/'camera/received.csv',
             C/'camera/summary.json', C/'cost/measurement.json', C/'orchestration.json',
             BASE/'ports.before.acl', BASE/'ports.after.acl']
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    hashes = {str(p.relative_to(BASE)):sha(p) for p in paths}
    read = lambda p: json.loads(p.read_bytes())
    rows = lambda p: list(csv.DictReader(p.open(newline='')))
    scans = rows(C/'c1-scans.csv')
    measured = [s for s in scans if s['measured']=='1' and s['result']=='0']
    per_scan = Counter()
    expected_index = Counter()
    with (C/'c1-points.csv').open(newline='') as stream:
        for point in csv.DictReader(stream):
            n=int(point['scan'])
            assert int(point['index'])==expected_index[n]
            expected_index[n]+=1
            per_scan[n]+=1
    assert all(per_scan[int(s['scan'])]==int(s['count']) for s in scans)
    c1time = [int(s['end_monotonic_ns'])/1e9 for s in measured]
    frames = [json.loads(line) for line in (C/'imu/frames.jsonl').open()]
    imu = read(C/'imu/summary.json')
    selected = [r for r in frames if r['phase']=='measure' and r['type']==1]
    assert dict(Counter(str(r['sensor_tag']) for r in selected))==imu['raw_tags']
    itime = [r['host_monotonic_ns']/1e9 for r in selected]
    offsets = [(r['host_wall_ns']-r['host_monotonic_ns'])/1e9 for r in frames]
    offset = statistics.median(offsets)
    # 실제 입력의 wall/monotonic 차가 일정한 범위에서만 측정 창 교집합을 낸다.
    assert max(offsets)-min(offsets)<.001
    sent,received = rows(C/'camera/sent.csv'), rows(C/'camera/received.csv')
    indexed = {int(r['stamp_ns']):r for r in sent}
    assert len(indexed)==len(sent)
    assert len({r['stamp_ns'] for r in received})==len(received)
    assert all(int(r['stamp_ns']) in indexed and
               all(r[k]==indexed[int(r['stamp_ns'])][k] for k in ('bytes','crc32'))
               for r in received)
    camera = read(C/'camera/summary.json')
    assert (len(sent),len(received))==(camera['sent'],camera['received'])
    ctime = [int(r['stamp_ns'])/1e9-offset for r in sent]
    spans = dict(c1=[c1time[0],c1time[-1]],imu=[itime[0],itime[-1]],camera=[ctime[0],ctime[-1]])
    start=max(s[0] for s in spans.values());end=min(s[1] for s in spans.values())
    orchestration = read(C/'orchestration.json')
    costjob=orchestration['jobs']['cost']
    cost=read(C/'cost/measurement.json')
    cost_overlap = start<=costjob['started_monotonic_s']<costjob['ended_monotonic_s']<=end
    result = dict(
        source_sha256=hashes,
        c1=dict(measured_scans=len(measured), points_count=stats([int(r['count']) for r in measured]),
                receive_hz=(len(c1time)-1)/(c1time[-1]-c1time[0]),
                receive_interval_ms=stats([1000*(b-a) for a,b in zip(c1time,c1time[1:])]),
                measured_sdk_errors=sum(r['result']!='0' for r in scans if r['measured']=='1'),
                startup_sdk_timeouts=sum(r['result']!='0' for r in scans if r['measured']!='1'),
                raw_point_rows_match=True),
        imu=dict(boot_count=imu['boot_count'], measured_tags=imu['raw_tags'],
                 crc_errors=imu['decoder'].get('crc_errors',0), sequence_gaps=len(imu['sequence_gaps']),
                 fifo_overrun=imu['overrun_observations_max'], inactive=imu['inactive_status_observed'],
                 rates={k:v['mcu_read_span_rate_hz'] for k,v in imu['streams'].items()},
                 host_read_gap_ms=stats([1000*(b-a) for a,b in zip(itime,itime[1:])])),
        camera={k:camera[k] for k in ('sent','received','producer_fps','plumbing_passed',
                 'pipeline_null','receiver_exit','copy_publish_ms','receive_age_ms')},
        measurement_spans_monotonic_s=spans, simultaneous_measurement_s=end-start,
        wall_minus_monotonic_spread_ms=1000*(max(offsets)-min(offsets)),
        cost_overlaps_all_measurements=cost_overlap,
        cost={k:cost[k] for k in ('calls','mismatches','source_unchanged','positive_ms','total_ms')},
        process_exit_codes={k:v['returncode'] for k,v in orchestration['jobs'].items()},
        acl_restored=(BASE/'ports.before.acl').read_bytes()==(BASE/'ports.after.acl').read_bytes(),
        sources_unchanged=all(sha(BASE/p)==h for p,h in hashes.items()),
        vehicle_actuation=False, acquisition_timestamps_verified=False,
        scope='Simultaneous stationary sensor diagnostics and one frozen CPU workload; no live controller, image recognition, actuator, synchronization, or installed calibration proof.')
    assert result['sources_unchanged'] and result['acl_restored'] and cost_overlap and end-start>40
    with (A/'audit.json').open('x',encoding='utf8') as stream:
        json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))


if __name__=='__main__':
    main()
