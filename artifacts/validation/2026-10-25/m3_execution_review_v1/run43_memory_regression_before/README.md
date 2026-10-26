# 위협 기록 덮어쓰기의 수정 전 회귀

실제 `ExecutionLateralTrial`을 사용한 좌·우 반례가 모두 실패했고 A 단독 유지 검사만 통과했다(2 failed, 1 passed, exit 1). 실패 이유는 사각에 남은 A의 불확실성을 B의 새 반환으로 덮어쓴 뒤 `released_to_baseline`이 발생했기 때문이다. `source_before.tar`·pytest.xml·stdout을 수정 전 근거로 보존했다.
