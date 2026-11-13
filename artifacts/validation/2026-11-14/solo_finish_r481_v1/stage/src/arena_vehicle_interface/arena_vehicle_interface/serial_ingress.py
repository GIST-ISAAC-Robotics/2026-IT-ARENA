"""유한 큐의 단일 UART 소유자. ROS/명령 판단 없이 원문과 읽기 시각만 전달한다."""
from dataclasses import dataclass
import queue
import threading
import time


@dataclass(frozen=True, slots=True)
class SerialReceipt:
    kind: str
    data: bytes
    monotonic_ns: int
    realtime_ns: int


class SerialIngress:
    """동기 serial 호출은 이 I/O 작업만 수행. 오류/큐 포화 시 종료하며 재연결하지 않는다."""
    def __init__(self, port, *, capacity=512, factory=None):
        if type(capacity) is not int or not 1<=capacity<=512:
            raise ValueError('queue capacity outside 1..512')
        self.port=port;self.factory=factory
        self.receipts=queue.Queue(maxsize=capacity);self.commands=queue.Queue(maxsize=16)
        self.ready=threading.Event();self.stopping=threading.Event()
        self.thread=None;self.error=None
        self.stats=dict(rx_bytes=0,tx_bytes=0,rx_blocks=0,tx_frames=0,queue_high_water=0,
                        queue_overflow=0,max_read_gap_ns=0,port_closed=False)

    def start(self):
        if self.thread is not None:raise RuntimeError('one connection per owner')
        self.thread=threading.Thread(target=self._run,name='serial-ingress',daemon=True)
        self.thread.start()
        if not self.ready.wait(3):
            self.close();raise TimeoutError('serial open timeout')
        if self.error:raise RuntimeError(self.error)

    def send(self, packet):
        if self.stopping.is_set() or self.error or not self.ready.is_set():raise RuntimeError('serial owner is unavailable')
        if type(packet) is not bytes or not 1<=len(packet)<=1544:raise ValueError('bounded immutable wire frame required')
        try:self.commands.put_nowait(packet)
        except queue.Full:raise RuntimeError('serial command queue full') from None

    def get(self, timeout=0.):
        try:return self.receipts.get(timeout=timeout)
        except queue.Empty:
            if self.error:raise RuntimeError(self.error)
            return None

    def close(self):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(2)
            if self.thread.is_alive():raise RuntimeError('serial owner failed to terminate')

    def _record(self, kind, data):
        receipt=SerialReceipt(kind,bytes(data),time.monotonic_ns(),time.time_ns())
        try:self.receipts.put_nowait(receipt)
        except queue.Full:
            self.stats['queue_overflow']+=1
            raise RuntimeError('serial receipt queue full; bytes not silently discarded') from None
        self.stats['queue_high_water']=max(self.stats['queue_high_water'],self.receipts.qsize())
        return receipt.monotonic_ns

    def _run(self):
        link=None
        try:
            if self.factory is None:
                import serial
                factory=serial.Serial
            else:factory=self.factory
            link=factory(port=None,baudrate=921600,timeout=.002,write_timeout=.2)
            link.dtr=False;link.rts=False;link.port=self.port
            link.open();self.ready.set();previous_read=None
            while not self.stopping.is_set():
                for _ in range(4):
                    try:packet=self.commands.get_nowait()
                    except queue.Empty:break
                    if link.write(packet)!=len(packet):raise RuntimeError('partial serial write')
                    self.stats['tx_bytes']+=len(packet);self.stats['tx_frames']+=1
                    self._record('tx',packet)
                # 고정 크기/2 ms timeout: 작은 읽기 사건의 무제한 큐 생성을 피한다.
                data=link.read(1024)
                if data:
                    stamp=self._record('rx',data)
                    if previous_read is not None:
                        self.stats['max_read_gap_ns']=max(self.stats['max_read_gap_ns'],stamp-previous_read)
                    previous_read=stamp
                    self.stats['rx_bytes']+=len(data);self.stats['rx_blocks']+=1
        except BaseException as exc:
            self.error=type(exc).__name__+': '+str(exc)
        finally:
            if link is not None:
                link.close();self.stats['port_closed']=True
            self.stopping.set();self.ready.set()
