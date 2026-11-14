"""호스트 연결·불변 wire·단일 I/O 검사. 물리 장치/ROS 노드는 별도다."""
from pathlib import Path
import queue
import sys
import time
import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT/'scripts', ROOT/'src/arena_vehicle_interface', ROOT/'tests'):
    if str(path) not in sys.path: sys.path.insert(0, str(path))

from runtime_actuation_link import ActuationMailbox, ActuationSerialIngress, RuntimeActuationLink
from receiver_owned_actuation_host import ReceiverLeaseTiming
from arena_vehicle_interface.actuation_wire import Decoder, encode
from test_serial_ingress import FakeSerial, wait_for


def wire(kind, seq=1):
    return encode(dict(type='command',request='test',command=dict(v=1,kind=kind,seq=seq)))


def link(enabled=True):
    return RuntimeActuationLink(host_epoch='epoch',client_id='host',
        timing=ReceiverLeaseTiming(100_000_000,100_000_000,1000,'synthetic declared rates'),
        max_speed_mps=2.,max_steering_rad=.45,max_lease_us=100_000,
        request_timeout_ns=200_000_000,drive_enabled=enabled)


def offer(**changes):
    return dict(type='offer',v=1,boot='boot',generation=0,lease=1,state='DISARMED',owner=None,
        lease_issued_us=250_000,lease_expires_us=350_000,output_ready=True,
        status_now_us=250_000,estop_held=False,reason='startup',
        motor_feedback_v1=dict(valid=False,sequence=None,capture_us=None,counts=None)) | changes


def observed(h,row,ns):
    return h.observe(row,receipt_ns=ns,processed_ns=ns+1)


def armed():
    h=link();observed(h,offer(),1_000_000_000)
    request=h.control('ARM',1_001_000_000)['envelope']['request']
    observed(h,dict(type='ack',request=request,accepted=True,reason='explicit_arm'),1_002_000_000)
    observed(h,offer(generation=1,lease=2,state='ARMED',owner='host'),1_003_000_000)
    return h


def test_mailbox_latest_drive_is_original_immutable_wire():
    q=ActuationMailbox();first,last=wire('DRIVE',1),wire('DRIVE',2)
    q.put_nowait(first);q.put_nowait(last)
    assert q.get_nowait()==last and q.counts['drive_replaced']==1
    with pytest.raises(queue.Empty):q.get_nowait()


def test_stop_purges_pending_controls_and_drive_and_does_not_queue_drive_behind():
    q=ActuationMailbox()
    for i in range(8):q.put_nowait(wire('ARM',i))
    with pytest.raises(queue.Full):q.put_nowait(wire('ARM',9))
    q.put_nowait(wire('DRIVE'));stop=wire('STOP');q.put_nowait(stop)
    assert q.counts['stop_purged_controls']==8 and q.counts['stop_purged_drive']==1
    with pytest.raises(RuntimeError,match='STOP'):q.put_nowait(wire('DRIVE'))
    assert q.get_nowait()==stop
    with pytest.raises(queue.Empty):q.get_nowait()


@pytest.mark.parametrize('packet',[b'',b'broken',bytearray(b'IA'),wire('DRIVE')*2,wire('DRIVE')[:-1],
    encode(dict(type='offer')),wire('UNKNOWN')])
def test_noncommand_noncanonical_or_partial_wire_rejected(packet):
    with pytest.raises((ValueError,TypeError)):ActuationMailbox().put_nowait(packet)


def test_single_io_owner_and_original_tx_receipt():
    fake=FakeSerial();io=ActuationSerialIngress('fake',factory=lambda **_:fake)
    try:
        io.start();packet=wire('STOP');io.send(packet)
        wait_for(lambda:bool(fake.sent))
        receipt=io.get(timeout=1.)
        assert receipt.kind=='tx' and receipt.data==packet
        assert receipt.monotonic_ns<=time.monotonic_ns()
    finally:io.close()
    assert len(fake.owners)==1 and fake.closed


def test_extended_offer_is_split_without_fabricating_zero_feedback():
    h=link();result=observed(h,offer(),1_000_000_000)
    assert result['parts'].feedback['source_valid'] is False
    assert result['parts'].feedback['motor_counts'] is None
    assert h.session.latest('offer').payload==result['parts'].compact
    assert h.session.arm_barrier is None


def test_dry_default_cannot_arm_or_drive_but_can_stop():
    h=link(False);observed(h,offer(),1_000_000_000)
    assert h.control('ARM',1_001_000_000)['envelope'] is None
    assert not h.drive(speed_mps=.2,steering_rad=0.,source=h.window(1_002_000_000),
                       now_ns=1_004_000_000)['command_created']
    assert Decoder().feed(h.control('STOP',1_005_000_000)['wire'])[0]['command']['kind']=='STOP'


def test_arm_ack_and_new_offer_then_source_drive_wire_preserves_age():
    h=armed();source=h.window(1_006_000_000)
    result=h.drive(speed_mps=.2,steering_rad=.1,source=source,now_ns=1_016_000_000)
    frame=Decoder().feed(result['wire'])[0]['command']
    assert frame['source_age_us']>=10_000 and frame['ttl_us']<100_000
    assert frame['lease']==2 and frame['generation']==1
    assert not h.drive(speed_mps=.2,steering_rad=0.,source=source,now_ns=1_017_000_000)['command_created']
    h.control('STOP',1_018_000_000)
    assert not h.drive(speed_mps=.2,steering_rad=0.,source=h.window(1_020_000_000),
                       now_ns=1_021_000_000)['command_created']


def test_boot_change_closes_and_never_rearms_automatically():
    h=armed();observed(h,offer(boot='other-boot'),1_010_000_000)
    assert h.closed
    assert h.control('ARM',1_011_000_000)['envelope'] is None
    assert h.control('STOP',1_012_000_000)['wire']


def test_stop_without_offer_and_after_clock_regression_still_prepared():
    assert link().control('STOP',1)['wire']
    h=armed();result=h.control('STOP',1)
    assert h.closed and result['wire']


def test_invalid_extension_and_processing_before_receipt_are_not_accepted():
    h=link()
    with pytest.raises(ValueError):observed(h,offer(extra=1),1_000_000_000)
    assert h.session is None
    h.observe(offer(),receipt_ns=1_000_000_000,processed_ns=999_999_999)
    assert h.closed
