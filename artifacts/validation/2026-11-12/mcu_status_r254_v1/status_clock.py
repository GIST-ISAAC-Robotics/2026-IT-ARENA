"""MCU 상태의 관측 전용 시계 연결. 원 시각/수신 시각을 함께 보존한다."""
from collections import Counter

STATES={'DISARMED','ARMED','ACTIVE','ESTOP_LATCHED','FAULT_LATCHED'}


class McuStatusClock:
    time_basis='minimum_receipt_offset_mcu_state_estimate'
    time_contract_verified=False

    def __init__(self, warmup_ns=250_000_000):
        if type(warmup_ns) is not int or warmup_ns<0:raise ValueError('invalid warmup')
        self.warmup_ns=warmup_ns
        self.boot=None;self.retired=set();self.counts=Counter()
        self.last_mono=self.last_ros_offset=None;self.clock_epoch=0
        self.clear()

    def clear(self):
        self.first_mcu=self.last_mcu=self.offset=self.last_stamp=self.generation=None
        self.clock_epoch+=1

    def convert_batch(self, rows, *, monotonic_ns, realtime_ns):
        if any(type(x) is not int or x<0 for x in (monotonic_ns,realtime_ns)):
            raise ValueError('invalid receipt clocks')
        ros_offset=realtime_ns-monotonic_ns
        if self.last_mono is not None and (monotonic_ns<self.last_mono or abs(ros_offset-self.last_ros_offset)>5_000_000):
            self.clear();self.counts['clock_reset']+=1
        self.last_mono=monotonic_ns;self.last_ros_offset=ros_offset
        segments=[]
        for row in rows:
            if (type(row) is not dict or row.get('type')!='offer' or type(row.get('v')) is not int or row['v']!=1
                or type(row.get('boot')) is not str or not 1<=len(row['boot'])<=64
                or row.get('state') not in STATES or type(row.get('estop_held')) is not bool
                or type(row.get('output_ready')) is not bool or type(row.get('reason')) is not str
                or not 1<=len(row['reason'])<=32
                or any(type(row.get(k)) is not int or row[k]<0 for k in ('generation','status_now_us','lease_issued_us'))
                or row['status_now_us']!=row['lease_issued_us']):
                self.counts['invalid']+=1;continue
            if not segments or segments[-1][0]!=row['boot']:segments.append((row['boot'],[]))
            segments[-1][1].append(row)
        result=[]
        for boot,items in segments:
            if boot in self.retired:
                self.counts['old_boot']+=len(items);continue
            if boot!=self.boot:
                if self.boot is not None:self.retired.add(self.boot)
                if len(self.retired)>64:raise RuntimeError('too many MCU restarts; start a new explicit session')
                self.boot=boot;self.clear();self.counts['boot_reset']+=1
            valid=[]
            for row in items:
                stamp=row['status_now_us']*1000
                if ((self.last_mcu is not None and stamp<=self.last_mcu)
                    or (self.generation is not None and row['generation']<self.generation)):
                    self.counts['nonmonotonic']+=1;continue
                if self.first_mcu is None:self.first_mcu=stamp
                self.last_mcu=stamp;self.generation=row['generation'];valid.append((stamp,row))
            if not valid:continue
            offset=monotonic_ns-valid[-1][0]
            self.offset=offset if self.offset is None else min(offset,self.offset)
            for mcu_ns,row in valid:
                if mcu_ns-self.first_mcu<self.warmup_ns:
                    self.counts['warmup']+=1;continue
                mapped=mcu_ns+self.offset+ros_offset
                if mapped<0 or (self.last_stamp is not None and mapped<=self.last_stamp):
                    self.counts['nonmonotonic']+=1;continue
                assert mapped<=realtime_ns
                self.last_stamp=mapped
                result.append(dict(v=1,boot=boot,generation=row['generation'],state=row['state'],reason=row['reason'],
                    estop_held=row['estop_held'],output_ready=row['output_ready'],now_us=mapped//1000,
                    mcu_now_us=row['status_now_us'],host_receipt_ns=realtime_ns,receipt_age_ns=realtime_ns-mapped,
                    time_basis=self.time_basis,time_contract_verified=False,clock_epoch=self.clock_epoch,
                    scope='observation_only_no_drive'))
                self.counts['published']+=1
        return result
