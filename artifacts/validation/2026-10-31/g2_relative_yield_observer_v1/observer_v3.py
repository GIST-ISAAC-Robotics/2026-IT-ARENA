"""복귀 띠 한 번의 양의 관측에서 가속하지 않고 기존 복귀 단계 전환까지 감속 유지."""
from observer_v2 import PersistentYieldObserver


class RejoinYieldObserver(PersistentYieldObserver):
    wait_for_rejoin=True
