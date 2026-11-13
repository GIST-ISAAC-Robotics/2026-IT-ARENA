# 수신기 임대를 쓰는 실제 호스트 준비 r391

상태: 독립 검토·208검사 통과 후 호스트 준비 모듈과 회귀를 공통 소스에 이관했다. 기존 strict ClockFit 검토 경로는 hook으로만 분리하고 새 receiver-owned 정책을 별도 subclass로 둔다. 공통 코드 이관은 마쳤으며 ROS·serial·장치 연결은 아직 없다.

수신기가 자신의 발급 시각에서 `TTL - source_age`를 차감하고 처리 시점에 만료를 검사하므로, 이미 받은 동일 lease와 원 source를 묶으면 절대 시계 offset 없이도 수명을 늘리지 않을 수 있다. 명시 rate 범위에 따른 age 올림·TTL 내림은 유지하며 실물 비율 보증으로 확대하지 않는다. 이 정책은 도착 성공/항상 수락을 보장하지 않는다. 상세 증명·반례는 REVIEW.md를 따른다.

기존 ARM의 ACK+새 세대 offer·source watermark·STOP 선점·연결/시계 세대를 재사용한다. prepare_drive는 실제 검토한 offer를 고정하고 immutable wire를 반환한다. age 계산 뒤 새 lease만 바꿔 끼우거나 원천 시각을 송신 시각으로 재작성하지 않는다. 동시 호출을 지원하는 객체가 아니며 단일 소유자를 전제한다.

새 검사는 임의 절대 offset/선언 rate 경계, 원 source 구간, 시간 역전·만료·ARM 전 입력, STOP·세대 변경, reference receiver의 큐 지연 거절, lease 재결합 반례를 포함한다. 기존 strict host 회귀도 같은 hook 변경본으로 수행한다. 소스/로그는 execution_manifest 및 verify_v1에 남긴다.

남은 실제 연결: 확장 offer의 명시 스키마 분리, ROS→단조 시각 원천 변환, 단일 serial 소유자·고정 wire 송신/종료 STOP, 실제 상태·encoder/IMU 전달. 계산 후보를 구동 가능한 통합 앱으로 표시하지 않는다.

## 확인 결과

`verify_v1`에서 새 후보와 기존 strict host/observer/source-time/offer-clock 회귀208개를 통과했고93파일 전후 해시가 일치했다. 별도 Python import에서 staged session 경로를 확인했으며 pytest 프로세스 내부의 각 import 경로까지 따로 기록한 것은 아니다. `integration.json`은 이관 소스 해시다. 실제 장치 출력·구동 가능 여부는 이 결과로 입증하지 않는다.
