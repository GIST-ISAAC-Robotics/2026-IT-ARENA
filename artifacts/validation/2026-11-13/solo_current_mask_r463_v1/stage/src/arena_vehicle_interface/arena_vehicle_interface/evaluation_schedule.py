"""보호된 출력의 평가 전용 20 ms 예약/ACK 계약. 기본 구동 경로에서는 사용하지 않는다."""
import json
import math

TOPIC = '/model/arena_car/eval/scheduled_command'
EVENT_TOPIC = '/model/arena_car/eval/scheduled_events'


class EvaluationSchedule:
    def __init__(self):
        self.seq = 0
        self.epoch = None
        self.last_stamp = None
        self.stopped = False
        self.failed = None
        self.pending = {}
        self.last_event = 0
        self.receipts = []

    def emit(self, kind, epoch, stamp, at, deadline, speed=0., steering=0.):
        self.seq += 1
        text = f'A1 {self.seq} {kind} {epoch} {stamp} {at} {deadline} {speed:.17g} {steering:.17g}'
        self.pending[self.seq] = {'kind':kind, 'epoch':epoch, 'stamp_us':stamp, 'apply_us':at,
            'deadline_us':deadline, 'speed':speed, 'steering':steering,
            'received':False, 'done':False, 'until':max(stamp,at)+20000}
        return text

    def stop(self, now):
        if self.epoch is None: return []  # 초기 잠금 상태: ARM 전에는 구동 허가/양수 예약이 없다.
        if self.stopped: return []
        self.stopped = True
        # pending은 ACK 감사용으로 남겨 cancellation 수신을 요구한다.
        return [self.emit('stop',self.epoch or 'none',now,0,0)]

    def status(self, status, guard, now, accepted):
        if self.failed or guard.blocked: return self.stop(now)
        if not accepted: return []
        epoch = f"{status['boot']}:{status['generation']}"
        if epoch != self.epoch:
            if status['state'] != 'ARMED' or guard.speed != 0:
                self.failed = 'missing_arm'; return self.stop(now)
            self.epoch=epoch; self.last_stamp=status['now_us']; self.stopped=False
            return [self.emit('arm',epoch,self.last_stamp,0,status['deadline_us'])]
        if self.stopped: return []
        stamp=status['now_us']
        if self.last_stamp is not None and stamp<=self.last_stamp: return []
        self.last_stamp=stamp
        at=stamp+20000
        if stamp%1000 or not stamp<=now<at<status['deadline_us']:
            self.failed='invalid_schedule_time'; return self.stop(now)
        return [self.emit('set',epoch,stamp,at,status['deadline_us'],guard.speed,guard.steering)]

    def ack(self, event):
        if type(event) is not dict or event.get('event_seq') != self.last_event+1:
            self.failed='ack_sequence'; return
        self.last_event=event['event_seq']
        self.receipts.append(event)
        if event.get('type') in ('fault','rejected'):
            self.failed='plugin_'+str(event.get('reason')); return
        seq=event.get('seq'); p=self.pending.get(seq)
        if p is None:
            self.failed='unknown_ack'; return
        if any(event.get(k)!=p[k] for k in ('kind','epoch','stamp_us','apply_us','deadline_us')) or any(
            not isinstance(event.get(k),(float,int)) or not math.isfinite(event[k]) or
            abs(event[k]-p[k])>1e-12 for k in ('speed','steering')):
            self.failed='ack_payload'; return
        kind=event.get('type')
        if kind=='received' and not p['received'] and not p['done']:
            if p['kind']=='set' and not p['stamp_us']<=event.get('sim_us',-1)<p['apply_us']:
                self.failed='late_receive_ack'; return
            p['received']=True
        elif kind in ('applied','armed','stopped','cancelled') and p['received'] and not p['done']:
            expected={'set':'applied','arm':'armed','stop':'stopped'}[p['kind']]
            if kind!=expected and not(kind=='cancelled' and p['kind']=='set' and self.stopped):
                self.failed='wrong_terminal_ack'; return
            if kind=='applied' and event.get('sim_us')!=p['apply_us']:
                self.failed='wrong_apply_step'; return
            p['done']=True
        else: self.failed='duplicate_or_invalid_ack'

    def watch(self, now):
        if any(not p['done'] and now>=p['until'] for p in self.pending.values()):
            self.failed=self.failed or 'ack_timeout'
        return self.stop(now) if self.failed else []


def audit_scheduled(sent, events, states):
    """송신본과 실제 ACK를 재구성해 정지 우선 취소를 포함한 완결성을 검증한다."""
    schedule=EvaluationSchedule()
    errors=[]
    for row in sent:
        fields=row['packet'].split()
        if len(fields)!=9 or fields[0]!='A1': errors.append('wire');continue
        _,seq,kind,epoch,stamp,at,deadline,speed,steering=fields
        expected=schedule.emit(kind,epoch,int(stamp),int(at),int(deadline),float(speed),float(steering))
        if int(seq)!=schedule.seq or expected!=row['packet']:errors.append('sent_sequence_or_encoding')
    # STOP로 취소된 명령은 단말 ACK로 인정하되 아래 STOP 선행 여부를 개별 검사한다.
    schedule.stopped=True
    for event in events: schedule.ack(event)
    stop_times=[e['sim_us'] for e in events if e.get('type')=='stopped']
    for event in events:
        if event.get('type')=='cancelled' and not any(t==event['sim_us'] for t in stop_times):
            errors.append('cancellation_without_stop')
    checks={'nonempty':bool(sent and events and states),'ack_protocol':not errors and schedule.failed is None,
            'all_terminal':bool(schedule.pending) and all(p['done'] for p in schedule.pending.values()),
            'sender_clean':bool(states) and all(s.get('failure') is None for s in states),
            'positive_applied':any(e.get('type')=='applied' and e.get('speed',0)>0 for e in events),
            'stop_observed':bool(stop_times)}
    return {'checks':checks,'passed':all(checks.values()),'errors':errors,'protocol_failure':schedule.failed,
            'sent':len(sent),'events':len(events),'applied':sum(e.get('type')=='applied' for e in events)}
