"""r394 검증된 PTY FakeMcu. r401 속도/조향 상한과 boot 이름만 명시 변경."""
import os
import select
import threading
import time
from arena_vehicle_interface.actuation_wire import Decoder,encode
from arena_vehicle_interface.actuation_contract import ActuationReceiver,ContractConfig
from arena_vehicle_interface.drive_feedback import MODE_MOTOR
from audit_actuation_offer_clock import KEYS as OFFER_KEYS


class FakeMcu:
    """기존 PC 수신기 참조 + 합성 stationary count. 실제 출력 HAL은 없다."""
    def __init__(self,master):
        self.master=master;self.stop=threading.Event();self.arm_seen=threading.Event();self.release_arm=threading.Event()
        self.decoder=Decoder();self.commands=[];self.events=[];self.error=None;self.pending=None
        self.rx=ActuationReceiver(ContractConfig(feedback_mode=MODE_MOTOR,ticks_per_rev=28,
            gear_ratio=2.5*81/17,wheel_radius_m=.0325,max_speed_mps=1.5,max_steering_rad=.4578436017036438),boot_id='r407-pty-only')
        self.sequence=0;self.started=time.monotonic_ns()
        self.thread=threading.Thread(target=self.run,name='fake-mcu-PTY',daemon=True)

    def write(self,row):
        packet=encode(row);sent=0
        while sent<len(packet):sent+=os.write(self.master,packet[sent:])
        self.events.append(dict(direction='to_host',monotonic_ns=time.monotonic_ns(),payload=row))

    def offer(self,now):
        full=self.rx.offer(now)
        feedback=self.rx.feedback
        row={key:full[key] for key in OFFER_KEYS-{'type','output_ready'}}
        row.update(type='offer',output_ready=True,status_now_us=now,estop_held=False,reason=self.rx.reason,
            motor_feedback_v1=dict(valid=full['feedback_valid'],
                sequence=None if feedback is None else feedback[0],
                capture_us=None if feedback is None else feedback[1],counts=None if feedback is None else feedback[2]))
        self.write(row)

    def accept(self,row,now):
        accepted,reason=self.rx.receive(row['command'],now)
        record=dict(monotonic_ns=time.monotonic_ns(),payload=row,accepted=accepted,reason=reason)
        self.commands.append(record)
        if row.get('request'):
            self.write(dict(type='ack',request=row['request'],accepted=accepted,reason=reason))
        self.offer(now)

    def run(self):
        next_offer=0
        try:
            while not self.stop.is_set():
                now=max(1,(time.monotonic_ns()-self.started)//1000)
                self.sequence+=1
                self.rx.update_motor_encoder(now,sequence=self.sequence,capture_us=now,motor_counts=0)
                self.rx.tick(now)
                if self.pending is not None and self.release_arm.is_set():
                    row,self.pending=self.pending,None;self.accept(row,now)
                if now>=next_offer:self.offer(now);next_offer=now+20_000
                if select.select([self.master],[],[],.005)[0]:
                    for row in self.decoder.feed(os.read(self.master,4096)):
                        self.events.append(dict(direction='from_host',monotonic_ns=time.monotonic_ns(),payload=row))
                        if row.get('type')!='command':raise ValueError('unexpected host envelope')
                        if row['command']['kind']=='ARM':self.pending=row;self.arm_seen.set()
                        else:self.accept(row,max(1,(time.monotonic_ns()-self.started)//1000))
        except BaseException as error:self.error=repr(error)
