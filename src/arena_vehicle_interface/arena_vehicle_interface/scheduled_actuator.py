"""평가 전용 GuardedSimActuator 출력 어댑터. AppliedGuard와 실물 계약은 그대로다."""
import json
import time
from std_msgs.msg import String
from arena_vehicle_interface.actuation_ros import GuardedSimActuator
from arena_vehicle_interface.evaluation_schedule import EvaluationSchedule, TOPIC, EVENT_TOPIC
from arena_vehicle_interface.node_lifecycle import run_node


class ScheduledSimActuator(GuardedSimActuator):
    def __init__(self):
        super().__init__()
        self.schedule=EvaluationSchedule()
        self.safety_reason=None
        self.safety_time=None
        self.safety_observed=False
        self.scheduled_pub=self.create_publisher(String,TOPIC,100)
        self.schedule_state=self.create_publisher(String,'/actuation/scheduled_status',100)
        self.create_subscription(String,EVENT_TOPIC,self.on_ack,1000)
        self.create_subscription(String,'/safety/status',self.on_safety,100)

    def on_safety(self,message):
        try:self.safety_reason=json.loads(message.data)['reason']
        except (ValueError,KeyError,TypeError):self.safety_reason='invalid_safety_status'
        self.safety_time=(self.now_us(),time.monotonic())
        if self.safety_reason=='clear':self.safety_observed=True
        # 보호 개입은 예약된 정상 감속이 아니다. 평가를 중단하고 즉시 STOP를 발행한다.
        if self.schedule.epoch and self.safety_observed and not self.schedule.stopped and self.safety_reason!='clear':
            self.schedule.failed='safety_intervention'
            for packet in self.schedule.stop(self.now_us()):self.scheduled_pub.publish(String(data=packet))

    def on_ack(self,message):
        try: self.schedule.ack(json.loads(message.data))
        except (ValueError,TypeError,KeyError): self.schedule.failed='malformed_ack'

    def output(self):
        now,wall=self.now_us(),time.monotonic()
        packets=[]
        while self.pending and self.pending[0]['now_us']<=now:
            status=self.pending.popleft()
            accepted=self.guard.update(status,now,wall)
            if self.guard.speed>0 and self.safety_reason!='clear':
                self.schedule.failed='positive_without_clear_safety'
            packets+=self.schedule.status(status,self.guard,now,accepted)
        self.guard.watch(now,wall)
        if self.schedule.epoch and self.safety_observed and not self.schedule.stopped and (self.safety_time is None or
            now-self.safety_time[0]>=100000 or wall-self.safety_time[1]>=self.guard.wall_timeout_s):
            self.schedule.failed='safety_status_timeout'
        if self.guard.blocked: packets+=self.schedule.stop(now)
        packets+=self.schedule.watch(now)
        if self.schedule.failed: self.guard.stop('evaluation_'+self.schedule.failed)
        for packet in packets:self.scheduled_pub.publish(String(data=packet))
        self.audit.publish(String(data=json.dumps({'now_us':now,'speed_mps':self.guard.speed,
            'blocked':self.guard.blocked,'reason':self.guard.reason,'epoch':self.guard.epoch,
            'last_status_us':self.guard.last_stamp})))
        self.schedule_state.publish(String(data=json.dumps({'now_us':now,'failure':self.schedule.failed,
            'last_seq':self.schedule.seq,'ack_event_seq':self.schedule.last_event,
            'pending':sum(not p['done'] for p in self.schedule.pending.values())})))


def main(args=None):run_node(ScheduledSimActuator,args=args)
