# 원 모터축 피드백의 주기 송신 r285

MCU 운영 연결 전에 `offer_motor_feedback_v1` 선택 송신을 추가했다. **기본 OFF**이며 `offer_status_v1`·모터축 모드가 함께 필요하다. 별도 저속 진단 대신 주기 offer에 원 취득 시각/순번/정수 카운트/유효 여부를 실을 수 있다. 수신 전에는 null/false이고 오래된 표본은 원 카운트/시각을 보존한 채 false다. 송신 중 새 값이 들어와도 이미 일부 보낸 프레임을 갱신하지 않는다.

호스트 `McuStatusClock`은 같은 MCU 상태의 시계 추정에 원 취득 시각을 연결해 관측 전용 `motor_feedback`을 만든다. 절대 취득 시각이 교정됐다는 뜻이 아니며 `time_contract_verified=false`다. 잘못된 피드백은 오류로 구별하고 유효0을 만들지 않는다. 독립 MCU 상태 관측은 보존한다. 새 피드백이 없는 기존 offer의 필드/출력은 그대로다.

- 관련 wire·부분 전송·status·기존 TX 회귀 **106개 통과**. 최적화/ASan·UBSan 두 빌드를 포함한다.
- 원 소스 계보 **220개 통과**. 이전 TX/clock 원문은 `before_sources.zip`과 기존 계약 해시에 대조했다. 신규 계약은 `config/tests/motor_feedback_r285_sha256.json`이다.
- 새 선택 송신을 사용하는 ESP32-S3 target object 컴파일 통과(text17,627B·data4B·bss13,856B). 펌웨어 전체 링크/실시간 메모리 여유/실물 검증은 아니다.
- 첫 부분 송신 검사는 시작 시 state가 offer보다 먼저 전송되는 순서를 잘못 가정해2개 실패했다. `first_fixture_failure.log`를 보존하고 offer가 시작된 뒤 카운트를 바꾸는 조건으로 고쳤다. 송신 정책은 이를 이유로 바꾸지 않았다.

**현재 보드 앱·flash·실제 센서/모터/서보는 조작하지 않았다.** 이 시험의 카운트는 합성값이다. 엔코더 실제 에지0 문제를 해결하거나 운영 운동 입력을 활성화한 결과가 아니다. 현재 `firmware_sensor_link`는 계속 output false·새 피드백 선택 OFF다. 다음은 실제 PCNT·구동 출력·비차단 조향과 이 계약의 연결이며 장착 후 엔코더 신호/부호·조향 환산을 확인해야 한다.
