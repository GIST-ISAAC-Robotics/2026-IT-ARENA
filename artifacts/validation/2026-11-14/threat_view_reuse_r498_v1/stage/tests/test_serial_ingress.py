import threading,time
from collections import deque

import pytest

from arena_vehicle_interface.serial_ingress import SerialIngress


class FakeSerial:
    def __init__(self, **kwargs):
        self.blocks=deque();self.sent=[];self.owners=set();self.closed=False;self.partial=False
    def open(self):self.owners.add(threading.get_ident())
    def close(self):self.owners.add(threading.get_ident());self.closed=True
    def read(self,n):
        self.owners.add(threading.get_ident());time.sleep(.002)
        return self.blocks.popleft() if self.blocks else b''
    def write(self,data):
        self.owners.add(threading.get_ident());self.sent.append(data)
        return 0 if self.partial else len(data)


def wait_for(predicate):
    end=time.monotonic()+2
    while not predicate():
        assert time.monotonic()<end
        time.sleep(.002)


def test_ingress_survives_main_stall_keeps_original_receipt_and_one_owner():
    serial=FakeSerial();serial.blocks.extend(bytes([i])*100 for i in range(64))
    io=SerialIngress('fake',factory=lambda **_:serial)
    try:
        io.start();io.send(b'IA-control')
        wait_for(lambda:io.stats['rx_blocks']==64)
        processed=time.monotonic_ns();rows=[]
        while (row:=io.get()) is not None:rows.append(row)
        received=[r for r in rows if r.kind=='rx']
        assert [r.data[0] for r in received]==list(range(64))
        assert processed-received[0].monotonic_ns>50_000_000
        assert all(a.monotonic_ns<=b.monotonic_ns for a,b in zip(received,received[1:]))
        assert serial.sent==[b'IA-control']
    finally:io.close()
    assert len(serial.owners)==1 and threading.get_ident() not in serial.owners
    assert serial.closed and io.stats['port_closed'] and io.error is None


def test_receipt_overflow_is_visible_and_closes_port():
    serial=FakeSerial();serial.blocks.extend([b'data']*10)
    io=SerialIngress('fake',capacity=1,factory=lambda **_:serial)
    io.start();wait_for(lambda:io.error is not None);io.close()
    assert io.stats['queue_overflow']==1 and serial.closed
    assert io.get().data==b'data'
    with pytest.raises(RuntimeError,match='receipt queue full'):io.get()


def test_partial_write_fails_without_automatic_resend():
    serial=FakeSerial();serial.partial=True
    io=SerialIngress('fake',factory=lambda **_:serial)
    io.start();io.send(b'command');wait_for(lambda:io.error is not None);io.close()
    assert serial.sent==[b'command'] and serial.closed
    with pytest.raises(RuntimeError):io.send(b'retry')


def test_open_failure_and_invalid_frame():
    def fail(**_):raise OSError('unavailable')
    io=SerialIngress('fake',factory=fail)
    with pytest.raises(RuntimeError,match='unavailable'):io.start()
    io.close()
    good=SerialIngress('fake',factory=lambda **_:FakeSerial())
    good.start()
    try:
        for data in (b'',bytearray(b'x'),b'x'*1545):
            with pytest.raises(ValueError):good.send(data)
    finally:good.close()
