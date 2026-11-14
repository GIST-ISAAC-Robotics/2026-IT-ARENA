"""신호 허가의 임의 시각 대신 다음 새 회전에 첫 제출을 맞춘다."""
import pytest
from arena_autonomy.decision_delivery import DecisionDelivery
from test_decision_delivery import Worker,data,poll


@pytest.mark.parametrize('phase',[0.,.02,.04,.06,.08])
def test_initial_phase_aligns_on_next_source_without_shorter_output_interval(phase):
    worker=Worker();gate=DecisionDelivery(worker,align_first_source=True)
    # 첫 요청 시 이미 받은 스캔이 0~80ms 묵었을 수 있다. 입력 시각을 갱신하지 않는다.
    initial=1.1+phase
    assert not gate.offer(data(initial,1.))
    assert not gate.offer(data(initial+.005,1.))
    assert gate.offer(data(1.2,1.1))
    worker.complete();first=poll(gate,1.22)
    assert first['kind']=='candidate' and first['reference_s']==1.2
    assert first['source_age_s']==pytest.approx(.12)
    assert gate.offer(data(1.3,1.2))
    worker.complete();assert poll(gate,1.31) is None
    assert poll(gate,1.32)['kind']=='candidate'


def test_reset_rearms_first_source_but_does_not_reuse_acceleration_budget():
    worker=Worker();gate=DecisionDelivery(worker,align_first_source=True)
    assert not gate.offer(data())
    assert gate.offer(data(1.2,1.1));worker.complete();poll(gate,1.22)
    gate.invalidate()
    assert gate.last_application==1.22
    assert not gate.offer(data(1.24,1.1))
    assert not gate.offer(data(1.25,1.1))
    assert gate.offer(data(1.3,1.2));worker.complete()
    assert poll(gate,1.31) is None
    assert poll(gate,1.32)['kind']=='candidate'


def test_new_boundary_scan_is_not_queued_while_old_generation_still_runs():
    worker=Worker();gate=DecisionDelivery(worker,align_first_source=True)
    assert not gate.offer(data())
    assert gate.offer(data(1.2,1.1));gate.invalidate()
    assert not gate.offer(data(1.3,1.2)) and worker.pending is None
    worker.inflight=None
    assert not gate.offer(data(1.3,1.2))
    assert gate.offer(data(1.4,1.3))
