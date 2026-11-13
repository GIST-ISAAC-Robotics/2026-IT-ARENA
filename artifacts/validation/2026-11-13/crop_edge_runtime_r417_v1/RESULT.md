# r417 경계 성분 후보 공통 연결 결과

2026-11-13. **관련183검사와 실제ROS 양 노드 생성1검사를 통과하고 공통8파일에 이관했다.** 기본값과 실물 운영 프로필은 OFF다. 주행 결과는 후속 r418에서 확인한다.

- v1은179통과/4실패였고 실제ROS 단계는 시작하지 않았다. 시험 코드가 `BypassHandoffFollow`의 `.partial_policy`를 직접 참조했지만 실제 소유자는 `.normal.partial_policy`였다. 두 실패는 AttributeError가 직접 남았으며 spawn 두 실패는 부모 EOF만 남았다. 제품에 별도 alias를 추가하지 않았다.
- [fixture v2](revision_fixture_v2/README.md)는 실제 내부 객체 접근과 시험 모듈 경로만 수정했다. 원 v1 코드·로그·manifest를 보존했고 r414 정책과 staged8파일은 바이트 동일하다. v2에서는 같은 spawn 검사도 통과했다.
- [결과](verify_native_v2/report.json)는183+1통과, 실패/오류/skip0이며 동결278입력 불변이다. 실제 계획·현재 검사·vision 자식의 정상 회수와 독립 보호 객체 분리, 원 센서 표현·frame·시각·epoch, `/drive`와 `/drive/safe` 연결을 확인했다. 노드 생성/콜백 확인이지 양수 구동·제어루프 주행 증명은 아니다.
- 원 README의 Humble 명령은 작성 오류다. 실제 실행은 WSL의 기존 **Jazzy**와 기존 overlay를 사용했다. 새 ROS 설치나 Jetson 변경은 없다.
- 주 에이전트가 제안 diff와 결과를 검토하고 기존 공통소스 hash를 대조한 뒤 테스트한 바이트 그대로 복사했다. [이관 영수증](integration.json)에8파일과 원/후 hash를 남겼다. r414의 전체 원 공백 성분 검사·소실 위협 검사·TTL·속도 제한은 변경하지 않았다.

새 선택은 verified native 시뮬레이션 전용이다. 기본 활성화·실물 허가·새 물성·보호 우회는 없으며 저장 입력 개선이 실제 지속 진행으로 이어지는지는 [r418](../crop_edge_drive_r418_v1/README.md)의 별도 제한 주행으로 판단한다.
