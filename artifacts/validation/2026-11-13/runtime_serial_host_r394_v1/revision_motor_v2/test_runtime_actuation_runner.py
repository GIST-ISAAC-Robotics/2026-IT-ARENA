"""r394 준비용 회귀: 실제 UART/ROS graph/장치 접근 없이 owner 경계 검사."""
import asyncio
from collections import deque
from concurrent.futures import Future
from dataclasses import asdict,replace
import io
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import pytest

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
sys.path.insert(0,str(HERE/'staged/scripts'))
sys.path.append(str(HERE.parent/'staged/scripts'))  # 동결 v1 core만 재사용.
for path in (ROOT/'scripts',ROOT/'src/arena_vehicle_interface',ROOT/'tests'):
    if str(path) not in sys.path:sys.path.append(str(path))

import run_runtime_actuation_host as runner
from arena_vehicle_interface.actuation_wire import Decoder,encode
from arena_vehicle_interface.serial_ingress import SerialReceipt
from arena_vehicle_interface.mcu_status_clock import McuStatusClock
from runtime_actuation_link import RuntimeActuationLink,ActuationSerialIngress
from receiver_owned_actuation_host import ReceiverLeaseTiming
from same_host_command_clock import SameHostClockBounds,SameHostCommandClock
from test_serial_ingress import FakeSerial

OFFSET=1_800_000_000_000_000_000


def policy():
    return runner.ClockPolicy(
        ReceiverLeaseTiming(100_000_000,100_000_000,1000,'synthetic test assumptions'),
        SameHostClockBounds(100_000_000,100_000_000,50_000_000,1_000_000,0,0,1000,True,
                            'synthetic equal-rate clocks; not hardware measurement'),
        100_000,200_000_000)


def offer(**changes):
    return dict(type='offer',v=1,boot='boot',generation=0,lease=1,state='DISARMED',owner=None,
        lease_issued_us=250_000,lease_expires_us=350_000,output_ready=True,
        status_now_us=250_000,estop_held=False,reason='startup',
        motor_feedback_v1=dict(valid=False,sequence=None,capture_us=None,counts=None)) | changes


class Times:
    def __init__(self):self.now=1_000_000_000;self.offset=OFFSET;self.mode=('ROS_TIME',False);self.watch=0
    def mono(self):return self.now
    def ros(self):return self.offset+self.now
    def watchdog(self):self.watch+=10_000_000;return self.watch
    def advance(self,ns=1_000_000):self.now+=ns


class FakeIngress:
    def __init__(self,clock):
        self.clock=clock;self.sent=[];self.rows=deque();self.closed=False;self.calls=[]
        self.auto_tx=False;self.auto_ack=False
    def send(self,wire):
        self.sent.append(wire);self.calls.append('send')
        if self.auto_tx:self.rows.append(SerialReceipt('tx',wire,self.clock.now,self.clock.ros()))
        if self.auto_ack:
            request=Decoder().feed(wire)[0]['request']
            self.rows.append(SerialReceipt('rx',encode(dict(type='ack',request=request,accepted=True,
                reason='software_stop')),self.clock.now,self.clock.ros()))
    def get(self,timeout=0.):
        if not self.rows:return None
        row=self.rows.popleft();self.calls.append('get_'+row.kind);return row
    def close(self):self.closed=True;self.calls.append('close')


def setup(*,enabled=True,ingress=None):
    p=policy();t=Times();events=[]
    engine=RuntimeActuationLink(host_epoch='host',client_id='client',timing=p.receiver_lease,
        max_speed_mps=2.,max_steering_rad=.45,max_lease_us=p.max_lease_us,
        request_timeout_ns=p.request_timeout_ns,drive_enabled=enabled)
    clock=SameHostCommandClock(host_epoch='host',ros_epoch='ros',bounds=p.same_host)
    uart=ingress or FakeIngress(t)
    owner=runner.RuntimeHostOwner(engine=engine,ingress=uart,policy=p,clock=clock,
        monotonic_ns=t.mono,ros_now_ns=t.ros,read_mode=lambda:t.mode,event=events.append,
        raw_rx=io.BytesIO(),raw_tx=io.BytesIO(),watchdog_ns=t.watchdog)
    return owner,t,uart,events


def received(owner,t,row,*,receipt_ns=None):
    ns=t.now if receipt_ns is None else receipt_ns
    owner.io.rows.append(SerialReceipt('rx',encode(row),ns,OFFSET+ns));owner.receive()


def written(owner,t,ticket,*,wire=None):
    owner.io.rows.append(SerialReceipt('tx',ticket.wire if wire is None else wire,t.now,t.ros()))
    owner.receive()


def ack(owner,t,ticket,*,accepted=True,reason=None,request=None):
    received(owner,t,dict(type='ack',request=ticket.request if request is None else request,
        accepted=accepted,reason=runner.CONTROL_REASONS[ticket.kind] if reason is None else reason))


def arm(owner,t):
    received(owner,t,offer());t.advance()
    ticket=owner.request_control('ARM');written(owner,t,ticket);t.advance()
    ack(owner,t,ticket);t.advance()
    received(owner,t,offer(generation=1,lease=2,state='ARMED',owner='client'))
    owner.poll_controls(t.now)
    assert ticket.result['success']
    return ticket


def write_policy(tmp_path,data=None):
    data=data or dict(schema='runtime_host_clock_policy_v1',**asdict(policy()))
    file=tmp_path/'policy.json';file.write_text(json.dumps(data));return file


def args(tmp_path):
    return runner.parser().parse_args(['--port','fake','--output',str(tmp_path/'new'),
        '--seconds','1','--clock-policy',str(write_policy(tmp_path)),
        '--max-speed-mps','1','--max-steering-rad','.4578436017036438'])


def test_cli_defaults_and_policy_are_explicit_and_device_free(tmp_path):
    a=args(tmp_path);p,record=runner.validate_args(a)
    assert not a.enable_drive_wire and not a.debug_imu_events and a.imu_topic=='/hardware_runtime/imu'
    assert a.motor_feedback_config is None
    assert p==policy() and record['declared']['same_host']['source_ttl_ns']==100_000_000
    assert not a.output.exists()


@pytest.mark.parametrize('field,value',[('seconds',0),('seconds',3601),('seconds',float('nan')),
    ('max_speed_mps',0),('max_steering_rad',float('inf')),('port',' '),
    ('imu_topic','/drive/safe'),('imu_topic','/actuation/stop'),('imu_topic','relative')])
def test_invalid_cli_rejected_before_any_device(tmp_path,field,value):
    a=args(tmp_path);setattr(a,field,value)
    with pytest.raises(ValueError):runner.validate_args(a)


def test_existing_output_never_overwritten(tmp_path):
    a=args(tmp_path);a.output.mkdir()
    with pytest.raises(ValueError,match='fresh'):runner.validate_args(a)


@pytest.mark.parametrize('mutation',[lambda d:d.update(extra=1),lambda d:d.update(schema='other'),
    lambda d:d['same_host'].update(source_ttl_ns=101_000_000),
    lambda d:d['receiver_lease'].pop('max_rate_error_ppm'),
    lambda d:d['same_host'].update(assume_no_unsampled_clock_step=False),
    lambda d:d.update(max_lease_us=0),lambda d:d.update(request_timeout_ns=True)])
def test_bad_policy_does_not_fallback(tmp_path,mutation):
    data=dict(schema='runtime_host_clock_policy_v1',**asdict(policy()));mutation(data)
    with pytest.raises((ValueError,TypeError)):runner.load_policy(write_policy(tmp_path,data))


@pytest.mark.parametrize('text',['{"x":1,"x":2}','{"x":NaN}','{"x":Infinity}'])
def test_policy_duplicate_and_nonfinite_json_rejected(text):
    with pytest.raises(ValueError):runner.strict_json(text)


def test_serial_factory_requests_exclusive_port(monkeypatch):
    captured=[];sentinel=object()
    def serial(**kwargs):captured.append(kwargs);return sentinel
    monkeypatch.setitem(sys.modules,'serial',NS(Serial=serial))
    assert runner.exclusive_serial_factory(port=None,timeout=.002) is sentinel
    assert captured==[dict(port=None,timeout=.002,exclusive=True)]


def test_initial_stop_no_session_future_has_own_deadline():
    o,t,uart,_=setup();future=Future();ticket=o.request_control('STOP',future=future)
    assert o.engine.session is None and not future.done()
    t.now=ticket.deadline_ns;o.poll_controls(t.now)
    assert future.result()['reason']=='timeout_unconfirmed' and not future.result()['success']
    assert len(uart.sent)==1 and not o.terminating


def test_initial_stop_requires_matching_written_receipt_and_ack():
    o,t,_,_=setup();ticket=o.request_control('STOP');t.advance()
    ack(o,t,ticket,request='unrelated');written(o,t,ticket);o.poll_controls(t.now)
    assert ticket.result is None
    ack(o,t,ticket);o.poll_controls(t.now)
    assert ticket.result['success'] and not ticket.result['physical_stop_verified']


def test_ack_without_matching_written_receipt_does_not_complete_service():
    o,t,_,_=setup();ticket=o.request_control('STOP');t.advance();ack(o,t,ticket)
    written(o,t,ticket,wire=encode(dict(type='command',request='other',command=dict(v=1,kind='STOP'))))
    o.poll_controls(t.now);assert ticket.result is None
    written(o,t,ticket);o.poll_controls(t.now);assert ticket.result['success']


@pytest.mark.parametrize('bad',[(False,'software_stop'),(True,'explicit_arm')])
def test_bad_ack_cannot_be_replaced_by_later_success(bad):
    o,t,_,_=setup();ticket=o.request_control('STOP');written(o,t,ticket);t.advance()
    ack(o,t,ticket,accepted=bad[0],reason=bad[1]);ack(o,t,ticket);o.poll_controls(t.now)
    assert ticket.result['reason']=='ack_rejected_or_reason_mismatch' and not ticket.result['success']


def test_pending_initial_stop_blocks_other_control_without_sending():
    o,t,uart,_=setup();o.request_control('STOP');received(o,t,offer())
    other=o.request_control('ARM')
    assert other.result['reason']=='stop_confirmation_pending' and len(uart.sent)==1


def test_disabled_arm_and_drive_do_not_send_but_stop_does():
    o,t,uart,_=setup(enabled=False);received(o,t,offer());t.advance()
    assert not o.request_control('ARM').result['success'] and not uart.sent
    o.capture_clock();t.advance();o.drive(source_ros_ns=t.ros()-1000,speed_mps=.1,steering_rad=0.)
    assert not uart.sent
    assert o.request_control('STOP').wire


@pytest.mark.parametrize('kind,before,after,reason',[
    ('ARM','DISARMED','ARMED','explicit_arm'),('CLEAR','ESTOP_LATCHED','DISARMED','explicit_clear'),
    ('DISARM','ARMED','DISARMED','explicit_disarm')])
def test_control_needs_correlated_ack_new_offer_and_written(kind,before,after,reason):
    o,t,_,_=setup();received(o,t,offer(state=before,owner='client' if before=='ARMED' else None));t.advance()
    ticket=o.request_control(kind);assert ticket.wire
    ack(o,t,ticket,reason=reason);o.poll_controls(t.now);assert ticket.result is None
    t.advance();received(o,t,offer(generation=1,lease=2,state=after,owner='client' if after=='ARMED' else None))
    o.poll_controls(t.now);assert ticket.result is None
    written(o,t,ticket);o.poll_controls(t.now)
    assert ticket.result['success'] and not ticket.result['physical_output_verified']


def test_arm_ack_without_new_offer_times_out_without_auto_retry():
    o,t,uart,_=setup();received(o,t,offer());t.advance();ticket=o.request_control('ARM')
    written(o,t,ticket);ack(o,t,ticket);t.now=ticket.deadline_ns;o.poll_controls(t.now)
    assert not ticket.result['success'] and len(uart.sent)==1 and o.engine.session.arm_barrier is None


def test_observation_frames_never_create_session_authority():
    o,t,_,events=setup()
    for kind in runner.OBSERVATIONS:received(o,t,dict(type=kind,arbitrary='diagnostic'))
    assert o.engine.session is None and o.counts['runtime_observations']==2
    assert all(not r['session_authority'] for r in events if r['event']=='runtime_observation')


def test_original_uart_receipt_not_processing_time_replacement():
    o,t,_,events=setup();original=t.now;t.advance(5_000_000)
    received(o,t,offer(),receipt_ns=original)
    receipt=o.engine.session.latest('offer')
    assert receipt.received.lower_ns==original and receipt.received.upper_ns==original
    row=next(r for r in events if r['event']=='control_received')
    assert row['receipt_monotonic_ns']==original and row['processed_monotonic_ns']==t.now


def test_drive_preserves_large_epoch_original_source_age_and_immutable_wire():
    o,t,uart,events=setup();o.capture_clock();arm(o,t);t.now=1_020_000_000
    source=OFFSET+1_006_000_000;o.drive(source_ros_ns=source,speed_mps=.2,steering_rad=.1)
    frame=Decoder().feed(uart.sent[-1])[0]['command']
    assert frame['kind']=='DRIVE' and frame['source_age_us']>=14_000 and frame['ttl_us']<100_000
    prepared=next(r for r in events if r['event']=='drive_prepared')
    assert prepared['source']['source_ros_ns']==source and not prepared['source']['source_retimed']
    assert bytes.fromhex(prepared['prepared']['wire']['wire_hex'])==uart.sent[-1]
    count=len(uart.sent);t.advance();o.drive(source_ros_ns=source,speed_mps=.2,steering_rad=.1)
    assert len(uart.sent)==count  # watermark: repeated source is not refreshed.


@pytest.mark.parametrize('source',[0,OFFSET-1,OFFSET+2_000_000_000])
def test_ordinary_source_rejection_skips_only_command(source):
    o,t,uart,events=setup();o.capture_clock();t.advance();o.capture_clock();t.advance()
    o.drive(source_ros_ns=source,speed_mps=.1,steering_rad=0.)
    assert not uart.sent and not o.terminating
    assert any(r['event']=='drive_source_rejected' and not r['clock_fault'] for r in events)


@pytest.mark.parametrize('fault',['mode','step','reverse'])
def test_actual_clock_fault_requests_termination(fault):
    o,t,uart,_=setup();o.capture_clock();t.advance();o.capture_clock();t.advance()
    if fault=='mode':t.mode=('ROS_TIME',True)
    elif fault=='step':t.offset+=10_000_000
    else:t.now-=5_000_000
    o.drive(source_ros_ns=t.ros(),speed_mps=.1,steering_rad=0.)
    assert o.terminating and not uart.sent
    uart.auto_tx=uart.auto_ack=True
    result=o.shutdown('fault');assert result['matching_stop_tx_receipt'] and result['stop_ack_confirmed']


def test_transient_gap_reanchors_without_stop_or_old_source_repackaging():
    o,t,uart,events=setup();o.capture_clock();t.advance();o.capture_clock()
    t.advance(60_000_000);o.capture_clock()
    assert not o.terminating and not o.clock.ready
    t.advance();o.capture_clock();t.advance()
    o.drive(source_ros_ns=t.ros()-1_500_000,speed_mps=.1,steering_rad=0.)
    assert o.clock.ready and o.clock.reanchors==1 and not o.terminating and not uart.sent
    assert any(r['event']=='drive_source_rejected' for r in events)


def test_now_inside_declared_read_error_skips_without_fabricating_clock_fault():
    o,t,uart,events=setup()
    bounds=replace(o.policy.same_host,host_timestamp_error_ns=1000)
    o.policy=replace(o.policy,same_host=bounds)
    o.clock=SameHostCommandClock(host_epoch='host',ros_epoch='ros',bounds=bounds)
    o.capture_clock();t.advance()
    o.drive(source_ros_ns=t.ros()-100_000,speed_mps=.1,steering_rad=0.)
    assert not uart.sent and not o.terminating and o.clock.fault is None
    assert events[-1]['reason']=='processing_now_within_capture_uncertainty'


def test_boot_change_closed_session_still_decodes_final_stop_tx_ack():
    o,t,uart,_=setup();arm(o,t);t.advance();received(o,t,offer(boot='newboot'))
    assert o.session_closed() and o.terminating
    uart.auto_tx=uart.auto_ack=True
    result=o.shutdown('boot_changed')
    assert result['matching_stop_tx_receipt'] and result['stop_ack_confirmed']
    assert uart.calls[-1]=='close' and uart.calls.index('close')>uart.calls.index('get_tx')


@pytest.mark.parametrize('tx,ack_ok',[(True,True),(True,False),(False,True),(False,False)])
def test_shutdown_confirms_only_exact_stop_and_never_claims_physical_stop(tx,ack_ok):
    o,t,uart,_=setup();uart.auto_tx=tx;uart.auto_ack=ack_ok
    result=o.shutdown('test')
    assert result['matching_stop_tx_receipt'] is tx
    assert result['stop_ack_confirmed'] is (tx and ack_ok)
    assert not result['physical_stop_verified'] and not result['already_writing_frame_cancelled']
    assert uart.closed and o.engine.closed and uart.calls[-1]=='close'
    assert result['elapsed_watchdog_ns']<=runner.SHUTDOWN_DRAIN_NS+30_000_000


def test_raw_uart_and_status_preserve_invalid_feedback_without_fake_zero():
    o,t,_,events=setup();o.status_clock=McuStatusClock(warmup_ns=0);statuses=[];o.publish_status=statuses.append
    row=offer();received(o,t,row)
    assert o.raw_rx.getvalue()==encode(row) and statuses
    status=statuses[-1]
    assert status['time_contract_verified'] is False and status['motor_feedback']['motor_counts'] is None
    assert not status['motor_feedback']['source_valid']
    assert any(r['event']=='control_received' and r['payload']['motor_feedback_v1']['counts'] is None for r in events)


@pytest.mark.parametrize('debug',[False,True])
def test_mixed_split_uart_passes_original_receipt_to_existing_fifo_clock(debug):
    from test_sensor_link_wire import imu
    o,t,uart,events=setup();o.log_imu_details=debug;calls=[];original=o.imu_clock.convert_batch
    def convert(rows,**kwargs):calls.append((rows,kwargs));return original(rows,**kwargs)
    o.imu_clock.convert_batch=convert
    packet=imu()+encode(offer())
    for chunk in (packet[:10],packet[10:]):
        uart.rows.append(SerialReceipt('rx',chunk,t.now,t.ros()));o.receive()
    assert o.raw_rx.getvalue()==packet and o.engine.session is not None
    assert len(calls)==1 and calls[0][1]==dict(host_monotonic_ns=t.now,ros_receipt_ns=t.ros())
    assert any(r['event']=='imu_received' for r in events) is debug
    assert o.counts['imu_received']==1
    assert o.counts['imu_published']==0  # FIFO warmup is not fabricated as fresh IMU.


class FakeNode:
    def __init__(self):self.pubs={};self.subs={};self.services={};self.timers=[]
    def create_publisher(self,kind,topic,qos):
        pub=NS(messages=[]);pub.publish=pub.messages.append;self.pubs[topic]=pub;return pub
    def create_subscription(self,kind,topic,callback,qos,**kwargs):self.subs[topic]=callback;return callback
    def create_service(self,kind,name,callback,**kwargs):self.services[name]=callback;return callback
    def create_timer(self,period,callback,**kwargs):self.timers.append((period,callback));return callback


def ros_bind(o,*,future_factory=Future):
    node=FakeNode()
    def imu():return NS(header=NS(stamp=NS(),frame_id=''),orientation_covariance=[0.]*9,
        linear_acceleration_covariance=[0.]*9,angular_velocity=NS())
    def joint():return NS(header=NS(stamp=NS(),frame_id=''),name=[],position=[],velocity=[],effort=[])
    runner.bind_ros(node,o,types=dict(Imu=imu,JointState=joint,String=NS,Drive=object,Trigger=object,sensor_qos=object()),
        callback_group=object(),future_factory=future_factory,imu_topic='/hardware_runtime/imu')
    return node


def test_fake_ros_topics_and_original_header_are_preserved():
    o,t,_,_=setup();node=ros_bind(o);calls=[];o.drive=lambda **kwargs:calls.append(kwargs)
    assert set(node.subs)=={'/drive/safe'} and set(node.pubs)=={'/hardware_runtime/imu','/actuation/status'}
    assert set(node.services)==set(runner.SERVICE_KINDS)
    node.subs['/drive/safe'](NS(header=NS(stamp=NS(sec=123,nanosec=456)),drive=NS(speed=.2,steering_angle=.1)))
    assert calls==[dict(source_ros_ns=123_000_000_456,speed_mps=.2,steering_rad=.1)]
    node.subs['/drive/safe'](NS(header=NS(stamp=NS(sec=123,nanosec=1_000_000_000)),drive=NS(speed=.2,steering_angle=.1)))
    assert len(calls)==1
    o.publish_imu(NS(stamp_ns=12_345,sample={'gyro_rad_s':[1.,2.,3.]}))
    msg=node.pubs['/hardware_runtime/imu'].messages[-1]
    assert msg.header.stamp.nanosec==12_345 and msg.orientation_covariance[0]==-1.
    assert msg.linear_acceleration_covariance[0]==-1. and msg.angular_velocity.z==3.


def test_fake_async_trigger_initial_stop_completes_false_at_own_timeout():
    async def scenario():
        o,t,_,_=setup();node=ros_bind(o,future_factory=asyncio.get_running_loop().create_future)
        task=asyncio.create_task(node.services['/actuation/stop'](object(),NS()))
        await asyncio.sleep(0)
        assert not task.done() and len(o.tickets)==1
        t.now=next(iter(o.tickets.values())).deadline_ns;o.poll_controls(t.now)
        response=await task
        assert response.success is False and json.loads(response.message)['reason']=='timeout_unconfirmed'
    asyncio.run(scenario())


def test_real_io_thread_with_fake_uart_drains_stop_before_close():
    fake=FakeSerial();ingress=ActuationSerialIngress('fake',factory=lambda **_:fake)
    o,t,_,_=setup(ingress=ingress);o.watchdog_ns=time.perf_counter_ns
    ingress.start()
    try:
        result=o.shutdown('fake_uart_only')
        assert result['matching_stop_tx_receipt'] and not result['stop_ack_confirmed']
        assert Decoder().feed(fake.sent[-1])[0]['command']['kind']=='STOP'
        assert len(fake.owners)==1 and fake.closed
        assert o.raw_tx.getvalue()==b''.join(fake.sent)
    finally:ingress.close()


@pytest.mark.parametrize('fault',[False,True])
def test_fake_ros_main_lifecycle_never_opens_device_and_flushes_shutdown(tmp_path,monkeypatch,fault):
    """실제 rclpy import/ROS graph 없이 main의 import·종료·로그 결합을 확인한다."""
    instances=[];nodes=[];ros_calls=[];real_mono=time.monotonic_ns;real_ros=time.time_ns
    class FakeIO(FakeIngress):
        def __init__(self,port,**kwargs):
            super().__init__(Times());self.stats={};self.error=None;instances.append(self)
            assert port=='fake' and kwargs['factory'] is runner.exclusive_serial_factory
        def start(self):self.calls.append('start')
        def send(self,wire):
            self.sent.append(wire);self.calls.append('send');ns=real_mono()
            self.rows.append(SerialReceipt('tx',wire,ns,real_ros()))
            request=Decoder().feed(wire)[0]['request']
            self.rows.append(SerialReceipt('rx',encode(dict(type='ack',request=request,accepted=True,
                reason='software_stop')),ns,real_ros()))
    class Node(FakeNode):
        def __init__(self,name):
            super().__init__();nodes.append(self);self.destroyed=False
            self.clock=NS(clock_type=NS(name='ROS_TIME'),ros_time_is_active=False,
                          now=lambda:NS(nanoseconds=real_ros()))
        def get_clock(self):return self.clock
        def get_parameter(self,name):assert name=='use_sim_time';return NS(value=False)
        def destroy_node(self):self.destroyed=True
    class Executor:
        def add_node(self,node):self.node=node
        def spin_once(self,timeout_sec):
            if fault:self.node.clock.ros_time_is_active=True
            self.node.timers[0][1]()
        def shutdown(self):ros_calls.append('executor_shutdown')
    modules={
        'rclpy':NS(init=lambda **kwargs:ros_calls.append(('init',kwargs)),shutdown=lambda:ros_calls.append('shutdown')),
        'rclpy.node':NS(Node=Node),'rclpy.executors':NS(SingleThreadedExecutor=Executor),
        'rclpy.callback_groups':NS(ReentrantCallbackGroup=object),
        'rclpy.signals':NS(SignalHandlerOptions=NS(NO='no_handlers')),
        'rclpy.task':NS(Future=Future),'rclpy.qos':NS(qos_profile_sensor_data=object()),
        'sensor_msgs.msg':NS(Imu=object,JointState=object),'std_msgs.msg':NS(String=NS),
        'std_srvs.srv':NS(Trigger=object),'ackermann_msgs.msg':NS(AckermannDriveStamped=object)}
    for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
    monkeypatch.setattr(runner,'ActuationSerialIngress',FakeIO)
    elapsed=iter([0.,.6,1.2]);monkeypatch.setattr(runner.time,'monotonic',lambda:next(elapsed))
    policy_path=write_policy(tmp_path);output=tmp_path/'result'
    monkeypatch.setattr(sys,'argv',['runner','--port','fake','--output',str(output),'--seconds','1',
        '--clock-policy',str(policy_path),'--max-speed-mps','1','--max-steering-rad','.45'])
    before={number:runner.signal.getsignal(number) for number in (runner.signal.SIGINT,runner.signal.SIGTERM)}
    if fault:
        with pytest.raises(SystemExit) as error:runner.main()
        assert error.value.code==1
    else:runner.main()
    result=json.loads((output/'summary.json').read_text())
    assert result['complete'] is (not fault) and result['shutdown']['matching_stop_tx_receipt']
    assert result['shutdown']['stop_ack_confirmed'] and not result['physical_stop_verified']
    assert not result['drive_wire_enabled'] and not result['source_timestamp_rewritten']
    assert len(instances)==1 and instances[0].closed and nodes[0].destroyed
    assert all(Decoder().feed(w)[0]['command']['kind']=='STOP' for w in instances[0].sent)
    assert instances[0].calls[-1]=='close' and (output/'uart_tx.bin').read_bytes()==b''.join(instances[0].sent)
    assert ros_calls[0][1]['signal_handler_options']=='no_handlers' and ros_calls[-1]=='shutdown'
    assert before=={number:runner.signal.getsignal(number) for number in before}


def motor_config():
    return dict(schema='runtime_motor_feedback_v1',counts_per_motor_revolution=28,
        gear_ratio=2.5*81/17,wheel_radius_m=.0325,encoder_sign=1,
        maximum_interval_ns=200_000_000,maximum_age_ns=200_000_000,maximum_speed_mps=5.)


def write_motor_config(tmp_path,data=None):
    path=tmp_path/'motor.json';path.write_text(json.dumps(motor_config() if data is None else data));return path


def test_optional_motor_config_is_exact_and_preserves_declaration(tmp_path):
    assert runner.load_motor_config(None)==(None,None)
    converter,record=runner.load_motor_config(write_motor_config(tmp_path))
    assert record['declared']==motor_config() and converter.spec.counts_per_motor_revolution==28
    assert converter.sign==1 and converter.previous is None


@pytest.mark.parametrize('mutation',[lambda d:d.update(schema='other'),lambda d:d.update(extra=1),
    lambda d:d.pop('encoder_sign'),lambda d:d.update(encoder_sign=0),lambda d:d.update(gear_ratio=0),
    lambda d:d.update(counts_per_motor_revolution=True),lambda d:d.update(wheel_radius_m=-1),
    lambda d:d.update(maximum_interval_ns=0),lambda d:d.update(maximum_age_ns=0),
    lambda d:d.update(maximum_speed_mps='5'),lambda d:d.update(maximum_speed_mps=False)])
def test_bad_motor_config_rejected_before_device(tmp_path,mutation):
    data=motor_config();mutation(data)
    with pytest.raises((ValueError,TypeError)):runner.load_motor_config(write_motor_config(tmp_path,data))


def test_main_rejects_bad_motor_config_before_ros_import_or_output(tmp_path,monkeypatch):
    path=write_motor_config(tmp_path,{'schema':'invalid'});policy_path=write_policy(tmp_path)
    monkeypatch.setitem(sys.modules,'rclpy',None)
    monkeypatch.setattr(sys,'argv',['runner','--port','never-open','--output',str(tmp_path/'result'),
        '--seconds','1','--clock-policy',str(policy_path),'--max-speed-mps','1',
        '--max-steering-rad','.45','--motor-feedback-config',str(path)])
    with pytest.raises(SystemExit) as error:runner.main()
    assert error.value.code==2 and not (tmp_path/'result').exists()


def test_optional_motor_joint_uses_same_status_conversion_and_original_stamp(tmp_path):
    o,t,_,events=setup();o.motor_feedback,_=runner.load_motor_config(write_motor_config(tmp_path))
    node=ros_bind(o);o.status_clock=McuStatusClock(warmup_ns=0)
    assert '/drive_motor/encoder' in node.pubs
    def sample(seq,count,stamp,valid=True):
        received(o,t,offer(lease=seq+1,lease_issued_us=stamp,lease_expires_us=stamp+100_000,status_now_us=stamp,
            motor_feedback_v1=dict(valid=valid,sequence=seq,capture_us=stamp-1000,counts=count)))
    sample(1,10,250_000)
    assert not node.pubs['/drive_motor/encoder'].messages
    t.advance(20_000_000);sample(2,10,270_000)
    stationary=node.pubs['/drive_motor/encoder'].messages[-1]
    raw=json.loads(node.pubs['/actuation/status'].messages[-1].data)['motor_feedback']
    assert stationary.velocity==[0.] and stationary.name==['drive_motor_shaft']
    assert stationary.header.frame_id=='drive_motor_encoder'
    assert stationary.header.stamp.sec*1_000_000_000+stationary.header.stamp.nanosec==raw['estimated_capture_ns']
    assert raw['motor_counts']==10 and not raw['time_contract_verified']
    t.advance(20_000_000);sample(3,11,290_000,False)
    assert len(node.pubs['/drive_motor/encoder'].messages)==1
    t.advance(20_000_000);sample(4,12,310_000)
    assert len(node.pubs['/drive_motor/encoder'].messages)==1
    t.advance(20_000_000);sample(5,13,330_000)
    moving=node.pubs['/drive_motor/encoder'].messages[-1]
    assert moving.velocity[0]==pytest.approx(2*3.141592653589793/28/.02)
    snap=o.motor_snapshot()
    assert snap['published']==2 and snap['first']['motor_counts']==10 and snap['last']['motor_counts']==13
    assert snap['publication_observed_rate_hz']==pytest.approx(1/.06)
    assert len([r for r in events if r['event']=='motor_feedback_summary'])==1
    assert o.motor_feedback.counts['published']==2  # convert exactly once per status.
