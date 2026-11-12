# r238 — 서로 다른 시계의 MCU 임대 토큰 연결

Jetson의 ROS 시각과 MCU 부팅 후 시각을 직접 빼던 시뮬 전용 전제를 명시 옵션으로 분리했다. 기본 `offer_time_policy=shared_clock`은 기존 그대로다. `receiver_owned_lease`는 최근 100 ms 이내 **호스트가 수신한 송신 후보**를 고른다. 그 토큰이 MCU에서 아직 유효하다는 판정은 아니다.

명령의 boot/generation/lease와 원래 source_age는 그대로 전송한다. MCU는 자신의 임대 테이블과 `min(lease_expires, lease_issued + ttl - age)`로 수락 여부와 기한을 결정한다. 호스트가 토큰을 늦게 받았어도 MCU에서 끝난 임대를 되살리지 않는다. 출력 준비·명시 ARM·ARM 이후 원천 입력·중복/역행 입력 거부는 유지한다. 이전 `CompactHostSession`의 시계 구간을 이용한 사전 보증 후보와 구별하며, 이번 옵션은 도착 시 유효함을 미리 보장하지 않는다.

- 초기 관련 검증: **94 passed**. 독립 시계 양방향 차이, 실제 수신기의 지연 토큰/원천 만료 거부, STOP 이후 구명령 거부, 원 ROS 콜백의 output_ready/중복 source 처리를 포함한다.
- 추가 기존 송신·링크·재시작·출처 회귀는 **394 passed** (`tests.log`).
- 시험은 합성 수신기/ROS 콜백이다. 실물 엔코더 정지 값을 생성하거나 `/drive/safe`를 직접 발행하지 않았다. 실제 UART 연결은 후속 r239에서 별도 확인한다.
- 시계 **시작점** 차이만 다룬다. 센서 취득 시각, 발진기 비율 오차, 실장 보정, 운용 구동 준비 완료를 뜻하지 않는다.

변경 전 소스는 `before_sources.zip`, 계보는 [r238 출처](../../../../config/tests/receiver_lease_host_r238_sha256.json)에 보존했다. 기본 주행·MCU TTL·물성 변경은 없다.
