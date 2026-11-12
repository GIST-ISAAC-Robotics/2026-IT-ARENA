"""기록된 센서 표본으로 현재 이력을 구성한다. 차체 정답/보간값은 쓰지 않는다.

observed는 표본을 포함한 최초 snapshot 시각 이후에만 공개한다.
delay20은 표본 취득+20ms 도착이라는 명시 가정이다. 원 기록의 실제 도착
시각을 복원했다고 주장하지 않는다. 스캔은 두 모드 모두 최초 기록 이후만 쓴다.
"""
import bisect
import copy
import math


class SensorTimeline:
    def __init__(self, events):
        snapshots=[]
        for event in events:
            if event['kind']!='call':continue
            data=event['data'];snapshots.append(data['input'])
            if data.get('current_rebase'):snapshots.append(data['current_rebase']['input'])
        snapshots.sort(key=lambda d:d['reference_s'])
        if not snapshots:raise ValueError('sensor snapshots required')
        if len({d['epoch'] for d in snapshots})!=1:
            raise ValueError('this bounded comparison requires one motion epoch')
        self.begin=snapshots[0]['reference_s']
        self.history_s=events[0]['data']['motion_config']['history_s']
        self.scans=[];scan_seen={};samples={k:{} for k in ('wheels','gyro')}
        self.snapshots=snapshots
        for data in snapshots:
            t=data['reference_s'];scan=data['scan']
            if scan.get('representation')!='native_measurements_v1':raise ValueError('native only')
            values=scan['measurements'];key=(values['source_epoch'],values['acquired_at_s'][0])
            if key in scan_seen:
                if scan_seen[key]!=scan:raise ValueError('one scan identity changed its values')
            else:
                scan_seen[key]=copy.deepcopy(scan);self.scans.append((t,copy.deepcopy(data)))
            for kind,history in data['history'].items():
                if len(history['times'])!=len(history['values']):raise ValueError('history length mismatch')
                for stamp,value in zip(history['times'],history['values']):
                    if not all(math.isfinite(v) for v in (stamp,value)) or stamp>t+1e-9:
                        raise ValueError('invalid/future source sample')
                    previous=samples[kind].get(stamp)
                    if previous is not None and previous[0]!=value:
                        raise ValueError('one sensor timestamp changed its value')
                    if previous is None:samples[kind][stamp]=(value,t)
        self.scan_refs=[t for t,_ in self.scans]
        epochs={d['scan']['measurements']['source_epoch'] for _,d in self.scans}
        starts=[d['scan']['measurements']['acquired_at_s'][0] for _,d in self.scans]
        if len(epochs)!=1 or any(b<=a for a,b in zip(starts,starts[1:])):
            raise ValueError('bounded replay requires one ordered sensor epoch')
        self.histories={k:sorted((stamp,value,seen) for stamp,(value,seen) in rows.items())
                        for k,rows in samples.items()}
        self.times={k:[r[0] for r in rows] for k,rows in self.histories.items()}
        self._cache_key=self._cache=None

    def current(self, now_s, *, mode):
        if mode not in ('held','observed','delay20'):raise ValueError('explicit timing assumption required')
        if not math.isfinite(now_s) or now_s<self.begin:raise ValueError('replay starts before first observation')
        tick=math.floor((now_s-self.begin+1e-10)/.02)
        reference=self.begin+tick*.02
        index=bisect.bisect_right(self.scan_refs,reference+1e-9)-1
        if index<0:return None
        # 같은 입력에서 반복 조회해도 시각을 새로 붙이지 않는다.
        key=(mode,tick,index)
        if key==self._cache_key:return copy.deepcopy(self._cache)
        data=copy.deepcopy(self.scans[index][1])
        if mode!='held':
            data['reference_s']=reference
            history={}
            for kind,rows in self.histories.items():
                lo=bisect.bisect_left(self.times[kind],reference-self.history_s)
                hi=bisect.bisect_right(self.times[kind],reference+1e-9)
                # 취득시각과 도착 가정을 분리한다. 모든 값/취득시각은 원문 그대로다.
                selected=[r for r in rows[lo:hi]
                          if (r[2] if mode=='observed' else r[0]+.02)<=reference+1e-9]
                history[kind]=dict(times=[r[0] for r in selected],values=[r[1] for r in selected])
            data['history']=history
            data['speed_mps']=history['wheels']['values'][-1] if history['wheels']['values'] else 0.
        self._cache_key=key;self._cache=copy.deepcopy(data)
        return data
