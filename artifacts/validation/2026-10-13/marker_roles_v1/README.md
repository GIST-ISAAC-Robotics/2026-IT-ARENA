# 마커 역할 검사 최초 실패

2026-10-13. 회귀 **19통과·66실패**, 종료 코드 1. `LocalPursuit.control()`의 실제 코드를 단순 `SimpleNamespace`에 적용했기 때문에 `super()`가 타입 오류를 냈다. 검출 결과나 주행 기능의 실패로 분류하지 않는다. 픽셀 행렬 CLI는 시작되지 않았다.

`tests.log`, `failed_source_snapshot.zip`에 최초 시험기/테스트를 보존했다. 후속은 ROS Node를 초기화하지 않은 실제 `LocalPursuit` 타입을 만들고 구동 메서드만 테스트 더블로 대체한다. 운영 소스/신호/마커/주행 규칙은 수정하지 않는다. [수정 후](../marker_roles_v2/README.md).
