# r491 실물 이관 준비 결과

2026-11-14. **최신 source-only bundle 1회 생성·검증 완료.** r492 회수 뒤 주 담당이 r491/r494/r495의 6파일을 공통 이관했고, 그 원본으로 [final_v1 ZIP](final_v1/runtime_handover_sources.zip)을 생성했다. 721파일/필수 누락0/전체 SHA·현재 공통 bytes·세 이관 after SHA가 모두 일치한다. ZIP은 로컬 보존·Git 제외다. [최종 manifest](final_v1/manifest.json) · [검증 영수증](final_v1/verification_receipt.json).

ZIP은 2,230,088 bytes, SHA-256 `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`다. manifest SHA-256은 `ee450fef0d971d2b94e75544420de5e13d6e9e694b4d2a12bdf6f848581098a5`다. CRC/내장 manifest/모든 파일 SHA와 공통 원문을 대조했다. 722 ZIP member 중 하나는 내장 source manifest다.

담당자는 Windows 파일 작업만 수행했다. WSL/Jetson/ROS/센서/ESP32/flash/ARM/구동 및 공통 파일 수정을 하지 않았다. 실제 영점·방향·PI·출력 설정은 미확정 `null`이다. r401 YAML은 참고 템플릿이며 공개 리소스도 자동 선택하지 않는다. 현장용 생성/선택/시작/STOP 명령은 [README](README.md)에 모았다. r491의 검토/통합 지연과 최초 검사 자료는 원본대로 보존했다.

공개 리소스 보완 완료: preview의 실제 기본 YAML 누락을 확인해 공개 hardware 1개·선택형 perception/빈 측정 서식 3개·최종 CAD 차량/센서 규격 2개를 명시 allowlist로 추가했다. 확장자 전체 허용이나 config 전체 수집은 하지 않는다. [resources_inventory_v2](resources_inventory_v2/manifest.json)는 r489 포함 721파일/필수 누락0, ZIP 없음이다. [resource_validation_v2.json](resource_validation_v2.json)은 실제 preview 인자 함수의 기본 경로 포함, 6파일 원문/SHA, 미검증/`null` 유지와 5개 파일 검사 exit0을 기록한다. 원 4검사 로그/최초 목록은 보존했다. host clock/motor JSON과 실제 board/zero/선택 YAML은 명시 제공 항목으로 남겨 자동 채택하지 않으며, CAD 변환기는 내장 Python 표를 사용하므로 기본 외부 CSV 의존이 없다. 공개 리소스 추가는 float32 두 생산 파일을 바꾸지 않았다.

## 해소한 빈곳

1. 과거 r404 ZIP에는 route 모듈·수동 패널/jog 모듈·설치 profile/limits·빌드 wrapper가 없다. 원 ZIP의 SHA와 누락 목록은 [file_validation.json](file_validation.json)에 보존했다. 새 [prepare 후보](stage/scripts/prepare_runtime_handover.py)는 r404 재귀 수집을 재사용하며 최신 전체 Python 패키지/의존 scripts/native 빌드 소스가 들어간다. 공통 A3 bundle을 운영용으로 재해석하지 않았다.
2. 최초 [inventory_v1/manifest.json](inventory_v1/manifest.json)은 714개 파일과 SHA를 기록하고 당시 r489 신규 모듈 하나가 아직 없음을 표시했다. 이후 r489와 r491/r494/r495 이관까지 반영한 최종 ZIP을 생성했다. 원 r404 ZIP·r401 템플릿은 보존한다.
3. 패널/jog .1 및 자율 운영 상한 .1은 원 발행식에서 float32 `.10000000149011612`가 되어 host의 정확한 `.1` 숫자 검사에 거절된다. [stage 내용 diff](speed_wire_content.diff)는 `development_jog.py`, `wall_follow.py`의 양수 발행값에 기존 `float32_limit_not_above`를 적용한다. `.09999999403953552`로 내려가며 STOP0/음수 기존 처리·host/MCU 보호 한도·조향은 바꾸지 않는다. [변경 SHA](speed_wire_changes.json)를 참조한다. 내부 계획값/진단값은 기존 의미를 유지하므로 실제 wire 값과 최대 1 ULP 다를 수 있다. 같은 두 경계의 모든 양수 송신에 적용되는 수치 직렬화 수정이며 신규 정책/장치 보호층은 아니다.

## 검증과 적용 경계

- `python -B .../test_prepare.py`: 4개 통과, exit0. 재귀 새 하위 모듈/native 소스, r489 미이관 표시, 캐시/바이너리/private 제외, 합성 ZIP의 SHA를 검사했다. 임시 합성 ZIP은 시험 종료 시 정리했다.
- `python -B .../test_speed_wire.py`: 1개 경계 검사 통과, exit0. stage의 **실제 두 발행식 AST**를 사용해 0/.05/.1의 ROS float32 표현을 검사하고 **실제 `ReceiverLeaseHostSession.prepare_drive`의 값 검사**까지 연결했다. 원 `.1`은 `invalid_drive_values`, 수정값은 숫자 검사 통과 뒤 의도한 `source_unavailable_after_value_gate`로 중단된다. 명령 생성/ARM/ROS 전송을 증명한 시험이 아니다. 음수 기존 값도 유지한다.
- stage 두 생산 파일 AST 파싱 성공, 원 공통 SHA 불변. [원본 로그](tests.log), [경계 로그](speed_wire_tests_v1.log), [영수증](file_validation.json). 새 장시간/전체 회귀를 추가하지 않았다.
- stage 복사 첫 시도는 CRLF의 `import time` 패턴 미일치로 쓰기 전에 중단됐다. LF로 읽어 내용 diff를 만들고 재시도했다. 공통 통합 시 기존 줄바꿈을 보존해야 한다. 이 준비 오류는 정책 시험 실패가 아니다.

## 이미 연결된 것 / 남은 것

실장 zero→profile header→`build_runtime_drive.sh --installed-profile` 선택은 기존 구현이 있다. 설치 limits는 두 ROS 조향과 host 인자를 연결하며 r489는 guard 선택까지 보완했다. 생성만으로 실제 설치/활성화되지 않는다. 수동/전체 패널 STOP→자식 취소+MCU STOP과 GPIO8→ESTOP latch→PWM0은 기존 소스/r480의 7검사로 확인돼 새 기능을 추가하지 않았다. 조립 후 실제 버튼·바퀴/서보 정지는 여전히 미확인이다.

지상 최초 목표 .05/명령 상한 .1은 명시 jog/host/board/수동 패널 설정으로 연결한다. 이전 r415의 .3/1.5 예시를 재사용하지 않는다. 자율 패널의 UI 최대는 운영 YAML `max_speed_mps`에서 정해지므로 수동 `manual_maximum_mps=.1`과 별개이며 이번 작업은 그 실물 적용 완료를 주장하지 않는다.

공중 13km/h 지시는 실측 과도 속도 상한 구현으로 자동 변환되지 않는다. 초기 점검에서 generator가 `max_wheel_mps`를 전달하지 않던 빈곳은 r495의 선택형 field로 연결돼 이번 ZIP에 포함된다. 기본은5m/s이며 실제 설치 threshold는 미선택이다. 기존 경계 초과 시 ARMED이면 `encoder_implausible` FAULT→목표0/PWM0이고, 주 담당의 [native receipt](../installed_feedback_limit_r495_v1/integration_v1/native_receipt.json)는 공통 header PC compile/run exit0을 기록한다. 명령 상한과 실측 타당성 한계·물리 관성을 구분하며, 실제 차단 경로 설정·확인 전 공중 고속시험은 보류한다. 상수를 낮추는 것만으로 절대 속도 보증이라고 하지 않는다.

모든 실행은 Windows 전경 명령으로 정상 종료했다. 남은 소유 시험 프로세스/장치 자원은 없다. 최신 ZIP 생성까지 완료해 자원을 주 담당에게 반환한다. 장착값을 기다려야 하는 profile 생성/빌드/설치/실물 구동은 별도이며, 이번 작업에서 하지 않았다.
