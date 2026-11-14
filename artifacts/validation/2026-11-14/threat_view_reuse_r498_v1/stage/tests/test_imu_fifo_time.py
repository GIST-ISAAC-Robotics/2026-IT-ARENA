import copy
import pytest
from arena_vehicle_interface.imu_fifo_time import FifoTimeSlots,fit_sensor_to_mcu


def row(seq,kind=4,slot=0,tick=1000,boot=7,**changes):
    tag=kind*8+slot*2;tag|=tag.bit_count()%2
    result=dict(type=1,boot=boot,sequence=seq,read_begin_us=seq*100+100000,
                read_end_us=seq*100+100020,fifo_status=0,tag_raw=tag,sensor_tag=kind,
                sensor_timestamp_raw=tick,raw_xyz=[12,-3,5],gyro_rad_s=[.1,-.2,.3])
    return {**result,**changes}


def test_same_slot_gyro_accel_preserve_sensor_tick_and_original_values():
    parser=FifoTimeSlots();parser.feed(row(0));g=row(1,1);a=row(2,2)
    before=copy.deepcopy(g);gyro=parser.feed(g);accel=parser.feed(a)
    assert gyro['sensor_tick']==accel['sensor_tick']==1000
    assert gyro['sample']==before and not gyro['ros_stamp_verified']
    gyro['sample']['raw_xyz'][0]=999
    assert g==before


@pytest.mark.parametrize('fault',('slot','parity','gap','overrun','format'))
def test_missing_or_invalid_timestamp_cannot_be_borrowed(fault):
    parser=FifoTimeSlots();parser.feed(row(0))
    bad=row(1,1)
    if fault=='slot':bad=row(1,1,slot=1)
    elif fault=='parity':bad['tag_raw']^=1
    elif fault=='gap':bad['sequence']=2
    elif fault=='overrun':bad['fifo_status']=0x40
    elif fault=='format':bad=row(1,10)
    assert parser.feed(bad) is None
    next_seq=bad['sequence']+1
    assert parser.feed(row(next_seq,2)) is None
    parser.feed(row(next_seq+1,slot=1,tick=1192))
    assert parser.feed(row(next_seq+2,1,slot=1))['sensor_tick']==1192


def test_new_boot_discards_previous_anchor_and_old_boot_cannot_return():
    p=FifoTimeSlots();p.feed(row(0));g=p.generation
    assert p.feed(row(0,1,boot=8)) is None
    assert p.generation>g
    p.feed(row(1,boot=8));assert p.feed(row(2,1,boot=8))
    assert p.feed(row(1,1,boot=7)) is None
    assert p.counts['old_boot']==1


def test_rollover_and_reset_are_distinct():
    p=FifoTimeSlots();p.feed(row(0,tick=2**32-100));generation=p.generation
    p.feed(row(1,slot=1,tick=92))
    assert p.feed(row(2,1,slot=1))['sensor_tick']==2**32+92
    assert p.generation==generation
    p.feed(row(3,tick=10));assert p.generation>generation


def test_duplicate_sequence_does_not_replace_current_slot():
    p=FifoTimeSlots();p.feed(row(0));assert p.feed(row(1,1))
    assert p.feed(row(1,tick=99999)) is None
    assert p.feed(row(2,2))['sensor_tick']==1000


def anchors():
    # 실제초당 tick 비율을 바꾼 합성. 25us를 고정하지 않는지 확인한다.
    return [dict(type=2,boot=7,active=True,sensor_timestamp_raw=t,
        read_begin_us=round(t*26.16+50000)-10,read_end_us=round(t*26.16+50000)+10)
        for t in (10000,50000,90000)]


def test_register_bracket_fits_rate_instead_of_assuming_25us():
    fit=fit_sensor_to_mcu(anchors(),boot=7)
    mapped=fit.map_interval(70000*25,boot='7')
    true=round((70000*26.16+50000)*1000)
    assert mapped.host_lower_ns<=true<=mapped.host_upper_ns
    assert min(float(v[0])*25 for v in fit.vertices)>26.15
    assert not mapped.ready_to_drive and not mapped.ros_stamp_verified


@pytest.mark.parametrize('fault',('boot','counter','nonaffine'))
def test_invalid_register_epoch_or_clock_is_rejected(fault):
    a=anchors()
    if fault=='boot':a[-1]['boot']=8
    elif fault=='counter':a[-1]['sensor_timestamp_raw']=1
    else:
        a[1]['read_begin_us']+=100000;a[1]['read_end_us']+=100000
    with pytest.raises(ValueError):fit_sensor_to_mcu(a,boot=7)


def test_mapping_rejects_outside_declared_extrapolation():
    fit=fit_sensor_to_mcu(anchors(),boot=7)
    with pytest.raises(ValueError):fit.map_interval(200000*25,boot='7')
