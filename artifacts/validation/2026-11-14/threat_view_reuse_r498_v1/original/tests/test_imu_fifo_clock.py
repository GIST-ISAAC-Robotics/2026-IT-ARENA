"""실제 시간/구동 승인과 분리한 FIFO 인과성 및 시계 전이 검사."""
import copy

from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock


def stream(count=700, boot=7):
    result=[]
    def append(row):
        result.append(dict(boot=boot, sequence=len(result), spi_errors=0, tx_errors=0, **row))
    for n in range(count):
        tick=10000+n*192
        capture=50000+tick*26.16
        if n%200==0:
            append(dict(type=2,active=True,sensor_timestamp_raw=tick,
                read_begin_us=round(capture)-10,read_end_us=round(capture)+10))
        for kind,lag in ((4,100),(1,800),(2,1000)):
            tag=kind*8+(n%4)*2;tag|=tag.bit_count()%2
            row=dict(type=1,sensor_tag=kind,tag_raw=tag,fifo_status=0,
                read_begin_us=round(capture+lag),read_end_us=round(capture+lag+50))
            if kind==4:row['sensor_timestamp_raw']=tick
            if kind==1:row['gyro_rad_s']=[.1,-.2,.3]
            append(row)
    return result


def convert(clock, rows, ros_shift=0):
    out=[]
    for row in rows:
        mono=50_000_000_000+row['read_end_us']*1000+1_000_000
        out.extend(clock.convert_batch([row],host_monotonic_ns=mono,
            ros_receipt_ns=mono+1_000_000_000_000+ros_shift))
    return out


def test_sensor_and_ros_time_are_estimates_with_original_read_anchor():
    rows=stream();before=copy.deepcopy(rows);clock=ImuFifoClock()
    out=convert(clock,rows)
    assert len(out)==500  # 두 번째 report 이후만, 미래 report로 초기 구간을 채우지 않음.
    assert rows==before and not clock.time_contract_verified
    assert all(x.register_anchor_sequence < x.sample['sequence'] for x in out)
    assert all(x.sample['gyro_rad_s']==[.1,-.2,.3] for x in out)
    assert all(700000 < x.read_based_stamp_ns-x.stamp_ns < 1000000 for x in out)
    assert all(a.stamp_ns < b.stamp_ns for a,b in zip(out,out[1:]))
    assert clock.read_clock.offset_ns==50_001_000_000


def test_prefix_results_do_not_depend_on_future_register_reads():
    rows=stream();split=next(i for i,r in enumerate(rows) if r['type']==2 and r['sequence']>1000)
    prefix=convert(ImuFifoClock(),rows[:split])
    whole=convert(ImuFifoClock(),rows)
    assert whole[:len(prefix)]==prefix


def test_ros_step_rewarms_and_does_not_reuse_old_ros_stamp():
    rows=stream();clock=ImuFifoClock()
    before=convert(clock,rows[:1202]);generation=clock.generation
    after=convert(clock,rows[1202:],ros_shift=50_000_000)
    assert before and after and clock.generation>generation
    assert clock.read_clock.counts['clock_reset']==1
    assert after[0].stamp_ns-before[-1].stamp_ns > 290_000_000


def test_sequence_loss_reacquires_timestamp_and_register_clock():
    rows=stream(1100);clock=ImuFifoClock()
    out=convert(clock,rows[:700]+rows[701:])
    assert clock.slots.counts['reset_sequence_gap']==1
    assert clock.counts['reset_sensor_epoch']>=2
    assert out and all(x.register_anchor_sequence < x.sample['sequence'] for x in out)
    assert not any(701 <= x.sample['sequence'] < 1803 for x in out)


def test_old_report_does_not_replace_fit_or_create_new_epoch():
    rows=stream();clock=ImuFifoClock();convert(clock,rows)
    fit=clock.fit;generation=clock.generation
    assert clock._sensor_interval(rows[0]) is None
    assert clock.fit is fit and clock.generation==generation


def test_fit_cannot_extrapolate_indefinitely_without_register_report():
    rows=stream(1400);clock=ImuFifoClock()
    # 전체 frame 순번은 연속이나 report가 아닌 metadata로 치환한다.
    rows=[dict(r,type=3) if r['type']==2 and r['sequence']>601 else r for r in rows]
    out=convert(clock,rows)
    assert out and out[-1].sample['sequence']<1600
    assert clock.counts['mapping_outside_window']>0


def test_new_boot_does_not_keep_previous_fit():
    clock=ImuFifoClock();old=stream(700);assert convert(clock,old)
    new=stream(100,boot=8)
    assert convert(clock,new)==[] and clock.fit is None
    assert convert(clock,old[-3:])==[] and clock.boot==8

