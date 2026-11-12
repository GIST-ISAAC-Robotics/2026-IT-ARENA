"""원 IO 수신 묶음 그대로 온라인 후보 재생. 미래 정보·장치 접근 없음."""
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_vehicle_interface')]
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock
from arena_vehicle_interface.imu_read_clock import ImuReadClock


def spread(v):
    x=sorted(v)
    return dict(count=len(x),min=x[0],p50=statistics.median(x),p95=x[int((len(x)-1)*.95)],max=x[-1]) if x else None


def run(rows):
    clock=ImuFifoClock();read=ImuReadClock();out=[];baseline=[];cost=[];fit_cost=[]
    batches=[]
    for r in rows:
        key=(r['host_monotonic_ns'],r['host_ros_ns'])
        if not batches or batches[-1][0]!=key:batches.append((key,[]))
        batches[-1][1].append(r)
    for (mono,ros),batch in batches:
        kwargs=dict(host_monotonic_ns=mono,ros_receipt_ns=ros)
        baseline.extend(read.convert_batch([r for r in batch if 'gyro_rad_s' in r],**kwargs))
        fits=clock.counts['fit_updated'];began=time.perf_counter_ns()
        out.extend(clock.convert_batch(batch,**kwargs))
        ms=(time.perf_counter_ns()-began)/1e6;cost.append(ms)
        if clock.counts['fit_updated']!=fits:fit_cost.append(ms)
    return clock,out,baseline,cost,fit_cost


def main():
    source=ROOT/'build/r254_recovered/run/capture/imu.jsonl'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    rows=[json.loads(x) for x in source.read_text().splitlines()]
    clock,out,base,cost,fit_cost=run(rows)
    by_seq={x.sample['sequence']:x for x in base}
    assert all(x.sample==by_seq[x.sample['sequence']].sample for x in out)
    assert all(x.register_anchor_sequence<x.sample['sequence'] for x in out)
    assert all(x.read_based_stamp_ns==by_seq[x.sample['sequence']].stamp_ns for x in out)
    assert all(x.stamp_ns<=x.sample['host_ros_ns'] for x in out)
    # 뒤쪽 입력을 주지 않아도 앞쪽 출력이 같은지 확인한다.
    boundary=next(i for i in range(len(rows)//2,len(rows)) if rows[i]['host_monotonic_ns']!=rows[i-1]['host_monotonic_ns'])
    _,prefix,_,_,_=run(rows[:boundary])
    assert prefix==out[:len(prefix)]
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    result=dict(source=str(source.relative_to(ROOT)).replace('\\','/'),source_sha256=digest,
        frames=len(rows),read_based_published=len(base),fifo_published=len(out),counts=dict(clock.counts),
        slot_counts=dict(clock.slots.counts),read_counts=clock.read_clock.counts,
        value_order_read_stamp_unchanged=True,prefix_equal_without_future_data=len(prefix),
        causal_register_anchors=True,first_sequence=out[0].sample['sequence'],
        sample_interval_ms=spread([(b.stamp_ns-a.stamp_ns)/1e6 for a,b in zip(out,out[1:])]),
        read_based_interval_ms=spread([(by_seq[b.sample['sequence']].stamp_ns-by_seq[a.sample['sequence']].stamp_ns)/1e6 for a,b in zip(out,out[1:])]),
        read_to_capture_shift_us=spread([(x.read_based_stamp_ns-x.stamp_ns)/1e3 for x in out]),
        mcu_interval_width_us=spread([(x.mcu_upper_ns-x.mcu_lower_ns)/1e3 for x in out]),
        callback_ms=spread(cost),fit_callback_ms=spread(fit_cost),
        source_to_host_time_verified=False,firmware_changed=False,drive_output=False,
        scope='stored actual receipt batches; causal tick mapping plus existing read/receipt host estimate; not ROS/live or installed calibration')
    (A/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (A/'mapped_gyro.jsonl').write_text(''.join(json.dumps(dict(sequence=x.sample['sequence'],stamp_ns=x.stamp_ns,
        mcu_lower_ns=x.mcu_lower_ns,mcu_upper_ns=x.mcu_upper_ns,anchor_sequence=x.register_anchor_sequence))+'\n' for x in out))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
