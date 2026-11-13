# Runtime 확장 offer의 명시 분리 어댑터 r392

2026-11-13. **Windows 준비 완료·실행 전이다.** 공통 소스/WSL/장치/ROS/serial은 수정하거나 실행하지 않았다. 기존 compact 검증을 완화하지 않고, 실제 firmware가 명시 선택한 확장 offer를 별도 경계에서 검증하여 분리한다.

## 형식과 연결

실제 writer는 [`portable/actuation_tx.hpp::write_offer_fields`](../../../../src/arena_vehicle_interface/portable/actuation_tx.hpp) 527–565행이다. `portable/src` 디렉터리는 없으며 구현은 header에 있다. [`firmware_runtime_drive`](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/main.cpp)와 [`firmware_runtime_preview`](../../../../src/arena_vehicle_interface/firmware_runtime_preview/main/main.cpp)는 `offer_status_v1`/`offer_motor_feedback_v1`을 둘 다 켠다. writer는 `status_now_us=lease_issued_us`, feedback은 원 `valid/sequence/capture_us/counts`를 기록한다.

[`staged/scripts/runtime_offer_adapter.py`](staged/scripts/runtime_offer_adapter.py)의 API:

```python
parts = adapt_runtime_payload(decoded_payload, max_lease_us=explicit_limit)
# RuntimePayloadParts(format, compact, status=None, feedback=None)
session.observe(parts.compact, received=original_receipt, connection_id=connection_id)
```

받는 형식은 세 가지뿐이다.

1. 기존 exact compact `offer/ack/state/diag`: 기존 `actuation_host_observer.validate_payload` 그대로 검증한다.
2. compact offer + 정확한 `status_now_us/estop_held/reason` 세 필드.
3. 위 status 확장 + 정확한 `motor_feedback_v1` 객체.

key 집합을 먼저 검사한 뒤 core를 추출하므로 unknown key를 조용히 제거하지 않는다. 불완전 확장, feedback-only, 다른 type, status 시각과 lease 발급 시각의 불일치, 범위/타입 오류, 미래 capture, 부분 null, `valid=true`의 무관측을 거절한다. 원 확장 payload 전체도1536-byte 한도를 검사한다. 잘못된 확장 일부만 버리고 core를 성공 반환하지 않는다. compact 자체의 기존 수용 범위는 바꾸지 않는다.

`status`와 `feedback`의 boot/generation/lease/status_now는 같은 한 offer에서 가져오므로 다른 상태 스냅샷과 합치지 않는다. status에는 원 state/reason/estop/output_ready를, feedback에는 source_valid/sequence/mcu_capture_us/motor_counts를 보존한다. 원본/분리 결과 사이 mutable 객체를 공유하지 않는다. 여러 packet 사이 boot/generation/lease 단조성·연결 ID·ARM 근거는 기존 Session/Observer의 책임이며 이 순수 함수가 별도 세션 상태기를 만들지 않는다.

feedback이 원래 없으면 `None`, 실제 무관측은 `source_valid=False`와 null 원값, 만료된 이전 표본은 `source_valid=False`와 그 원 count/capture를 반환한다. **0속도를 만들지 않는다.** signed count를 보존하고 기어비/바퀴 반경/속도를 계산하지 않는다. status/feedback을 새로운 offer 수신 사건으로 Session에 다시 넣어 원 receipt나 lease 나이를 갱신해서는 안 된다.

## 기존 코드 재사용과 경계

프레임/CRC/중복 JSON key는 기존 `actuation_wire.Decoder`가 담당한다. 어댑터는 그 뒤의 payload 경계만 다룬다. [`decode_motor_feedback`](../../../../src/arena_vehicle_interface/arena_vehicle_interface/motor_feedback_observation.py)는 이미 host로 추정한 상태 시각을 필수로 요구한다. 여기서는 host 시각을 꾸며 넣지 않고 그 함수와 같은 원 관측 shape/정수/capture 조건만 검사한다. 후속 관측 경로에서 실제 mapped 상태를 확보하면 기존 함수를 그대로 사용할 수 있으며, 준비 테스트는 양쪽의 원 필드 일치도 대조한다.

[`bridge_sensor_link_preview.py`](../../../../scripts/bridge_sensor_link_preview.py)의 `output_ready is False` 제약은 출력 비활성 preview 프로그램의 정책이다. 어댑터는 운영 형식의 `True`도 보존한다. 이 값이 관측 시각·피드백의 유효성이나 ARM 허가를 보장한다는 의미는 없다.

같은 IA 프레임으로 오는 `runtime_drive_observation_v1` 등 별도 observation type은 이 어댑터가 거절한다. 단일 serial 소유자의 상위 명시 라우터에서 별도 진단으로 처리해야 한다. 그 라우터·ROS source→단조 시계 변환·serial 연결은 이번 범위 밖이다.

## 준비한 검사와 고정 명령

[`test_runtime_offer_adapter.py`](test_runtime_offer_adapter.py)는33개 경우를 준비했다. 세 형식과 ready true/false, raw invalid/만료 표본, unknown/partial/nested 충돌, 시간/정수 반례, 기존 motor decoder와 원값 일치, 실제 Decoder→adapter→CompactHostObserver 연결, 원본/결과 aliasing, 전체 payload 크기를 검사한다. 원 compact validator가 확장 payload를 계속 거절하는지도 확인한다.

프로젝트 루트에서 실행 담당자가 한 번 수행한다. 이 문서 작성자는 실행하지 않았다.

```sh
python3 -m pytest -q artifacts/validation/2026-11-13/runtime_offer_adapter_r392_v1/test_runtime_offer_adapter.py
```

AST 구문·문서 링크·해시만 Windows에서 확인했다. 준비 파일과 검토 원 소스 해시는 `manifest.json`, 정적 결과는 `static_checks.json`에 기록한다. 실제 writer 바이트를 새로 수신하거나 빌드한 결과가 아니며, 새 backend/보드 프로필/구동 정책을 적용하지 않았다.

## 실행·이관 결과

`verify_v1`의 새 adapter33개와 공통 r391/기존 호스트208개, 총241개가 통과했고 동결 입력은 일치했다. adapter 구현과 artifact 의존을 제거한 테스트를 공통에 이관했다. 원 staged 검사를 덮어쓰지 않았으며 공통 최종 결합 검사는 후속 연결 묶음에서 수행한다. ROS/serial/장치 출력은 이 검사 범위가 아니다.
