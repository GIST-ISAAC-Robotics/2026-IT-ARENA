"""r522 시험 한정: 전체 payload를 순회/동결/저장하지 않는 기록 sink."""
from collections import Counter


class TraceOff:
    def __init__(self):
        self.closed=False
        self.events=Counter()
        self.reset_reasons=Counter()
        self.discard_reasons=Counter()

    def emit(self,kind,data):
        if self.closed:return
        self.events[kind]+=1
        if kind=='reset':self.reset_reasons[data.get('reason')]+=1
        elif kind=='call':
            delivery=data.get('delivery',{})
            if delivery.get('kind')=='discarded':self.discard_reasons[delivery.get('reason')]+=1

    def record_call(self,*args,**kwargs):
        self.events['record_call']+=1

    def close(self):
        self.closed=True

    def snapshot(self):
        return dict(mode='test_only_payload_off_after_validated_creation',events=dict(self.events),
            reset_reasons=dict(self.reset_reasons),discard_reasons=dict(self.discard_reasons),
            full_input_payload_recorded=False,control_snapshot_and_ipc_unchanged=True)
