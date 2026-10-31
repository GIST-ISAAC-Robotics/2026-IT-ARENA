# 방향별 초기 기준의 명시 연결

2026-10-31. [오프라인 후보](../g2_directional_reference_v1/PLAN.md)의43검사·세 초기 입력 비교를 근거로 `ready_directional_pair_v2`를 선택형으로 연결한다. 기존 `off`와 `ready_current_pair_v1`은 보존한다.

변경 파일은 초기 기준 공급자와 G1/G2 인계 생성자 두 곳이다. 기본값·현재 이동 영역·STOP·시각/READY 조건·물성을 바꾸지 않는다. 새 클래스는 검증 후보와 AST가 동일하고 이전 CurrentPairReference 클래스도 그대로임을 검사한다. 과거 소스는 before_connection.zip과 후속 해시 계약에 보존한다.

관련 회귀·빌드/설치 바이트 대조 후 정상1랩 저장 입력7815호출을 재생한다. 이미 검증된 ready_current_pair_v1의 저장 명령과 비교하여 추가 명령 변경 유무를 확인한다. 기존 입력과 후보 출력·footer·해시를 보존하며, 새 클래스의 반복성/실차 성공으로 확대하지 않는다. 차이가 있으면 결과를 보존하고 원인을 먼저 분석한다.
