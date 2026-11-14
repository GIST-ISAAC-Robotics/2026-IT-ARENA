"""단일 executor 전용: 이미 준비된 운동 표본을 원 callback으로 제한 소비한다.

이 파일은 r508 독립 stage 후보다. 원 시각/검증/큐의 순서를 유지하며,
입력을 기다리거나 버리지 않는다. 병렬 executor와 함께 쓰지 않는다.
"""
import time


def drain_ready(subscriptions, *, budget_s=.002, max_messages=64, clock=time.monotonic):
    started=clock()
    counts=[0]*len(subscriptions)
    total=0
    exhausted=False
    while total<max_messages and clock()-started<budget_s:
        progress=False
        for i,sub in enumerate(subscriptions):
            if total>=max_messages or clock()-started>=budget_s:
                exhausted=True
                break
            # Humble Executor._take_subscription와 같은 take 경로. 변환 뒤
            # 기존 wrapper/엄격 검증을 전부 거친다. stamp/공백을 고치지 않는다.
            with sub.handle:
                item=sub.handle.take_message(sub.msg_type,sub.raw)
            if item is not None:
                sub.callback(item[0])
                counts[i]+=1
                total+=1
                progress=True
        if not progress:break
    elapsed=clock()-started
    return dict(counts=counts,total=total,duration_ms=elapsed*1000,
                bounded_stop=exhausted or total>=max_messages or elapsed>=budget_s)
