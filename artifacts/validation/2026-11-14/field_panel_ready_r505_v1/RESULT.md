# r505 현장 패널 연결 상태 보완

2026-11-14. Windows 로컬 DOM·HTTP 응답 모형과 순수 Python 검사만 수행했다. ROS/WSL/Jetson/센서/구동 장치 및 외부 포트에 접근하지 않았으며 미리보기 서버나 백그라운드 프로세스를 시작하지 않았다.

## 발견과 수정

기존 패널은 재연결 시 이전 경기 바인딩과 속도 편집값을 유지했다. 이전 연결의 늦은 상태 응답이 새 차량 이름·조회 권한·서버 상한을 덮었고, 같은 연결에서도 먼저 보낸 WAIT_GREEN 응답이 늦게 도착하면 이미 수신한 주행 화면의 수동출발 버튼을 다시 활성화했다. HTTP 401/409에서는 상태 문구만 바뀌고 기존 버튼이 남았다. 서버의 바인딩/WAIT_GREEN 검사는 계속 존재하므로 이 현상이 실제 출발 허가나 잘못된 구동을 증명하지는 않는다.

`race_ops_page.py`에서 현재 인증 객체와 상태 조회 순번을 확인하여 오래된 응답·실패를 무시한다. 새 연결은 바인딩/서버 상한의 초기 수신을 기다리고, 조회 실패는 마지막 성공 시각을 무효화하여 기존 화면 조작을 비활성화한다. 이전 연결의 명령 응답도 새 화면 결과를 덮지 않는다. STOP은 최신 상태 바인딩 없이 요청할 수 있다. planner·guard·MCU·서버 상태 전이와 주행 조건에는 변경이 없다.

## 검증

| 구분 | 결과 | 근거 |
|---|---|---|
| 수정 전 실제 script 재현 | 새 8개 중 7실패·1통과 | `before.log`, `before.xml` |
| 수정 후 연결·지연응답·단위/펄스 DOM 회귀 | 8통과 | `final_page.log`, `final_page.xml` |
| 관련 운영/수동 준비·STOP 순수 회귀 | 33통과 | `related.log`, `related.xml` |
| 수정 파일 공백/diff 확인 | 통과 | `git diff --check` |

Node.js v24.18.0에서 실제 패널 JavaScript를 로컬 DOM/지연 HTTP 응답 모형으로 실행했다. Python 3.14.6에서 기존 로컬 pytest 묶음을 사용했다. 외부 의존성 설치는 없었다. 초기 수정의 8통과는 `after.*`, 마지막 응답 수신 예외 처리 검토 후 8통과는 `final_page.*`에 각각 보존했다.

재현 진입점은 `tests/test_operator_panel_page.py`이며 `tests/operator_panel_page_checks.js`를 실행한다. Node가 없는 환경에서는 해당 검사만 skip된다. 관련 검사는 `tests/test_race_ops.py tests/test_operator_panel_manual.py`다. `source_manifest.json`에 최종 수정 파일 해시와 환경/검증 요약을 보존했다.

## 현장 사용과 남은 경계

- 기존 `race_ops` HTTP 주소에 차량 이름·토큰으로 연결한다. 속도 상한은 m/s↔km/h 변환을 지원하며 STOP+MCU 비ARM에서 적용한다. 표시된 서버 적용값을 확인한 뒤 준비한다.
- 조립 후 사각 공간의 현행 첫 시험 지시는 상한0.1m/s(0.36km/h), 목표0.05m/s(0.18km/h)의 짧은 단발이다. 패널의 기존 목표 입력 기본값은0.15m/s이므로 현장 지시값으로 직접 편집한다. 이번 변경에서 기본값/상한을 바꾸지 않았다.
- 수동 시험은 기존 별도 `operation_mode:=manual` 구성에서 `수동 준비`→속도/시간/중앙 등가 바퀴각 입력→`입력값으로 실행`을 사용한다. 예를 들어0.05m/s×0.5s는 표시상0.025m다. 이는 실측 이동거리·정지거리가 아니다. 종료 뒤 다시 실행하려면 다시 준비한다.
- `신호 확인 후 수동 출발`은 자율 구성의 최초 WAIT_GREEN 상태에서만 사용한다. 이미 출발한 경기의 STOP 후 재개는 기존 START다. 운영자 신호 확인으로 기록하며 MCU·입력·경로·독립 보호 조건을 생략하지 않는다.
- STOP은 상태 조회 없이도 API 요청할 수 있지만 인증·통신·MCU 응답과 실제 차체 정지는 별개다. 실물 모터/서보/브레이크/물리 STOP·무선 AP 및 브라우저 시각 검수는 이번 검증에 포함하지 않았다.

공통 코드 수정은 패널 페이지 한 파일과 신규 회귀 두 파일뿐이다. 문서 현황·계획·활동일지는 주 담당자가 통합한다. 커밋·푸시는 하지 않았다.

## 독립 검토 후속: 재연결 중 전역 STOP의 수동 전달

재연결 직후 `last=null`이면 전역 STOP은 `/api/command`로 전송된다. 현재 `Handler.do_POST`가 이를 `OpsCore.submit(view=None)`으로 접수하고, `RaceOps.execute`는 수동 구성에서도 `stop`만 예외 허용한다. 실행 순서는 `manual.stop`→`manual_prepare=None`→경기 STOP 발행→MCU STOP 서비스 호출이다. 따라서 별도 `/api/manual` 응답을 기다릴 필요가 없다.

`verify_reconnect_stop.py`에서 실제 Handler/RaceOps 메서드와 기존 로컬 fake fixture를 사용하여 준비 중/준비 완료/단발 큐 대기/단발 실행 중의4경우를 확인했다. 모두 HTTP200 접수·수동 준비 해제·큐 비움·MCU STOP 호출을 확인했고, 실행 중에는 자식 stdin `STOP`·close와 모의 자식 회수까지 확인했다. `reconnect_stop_result.json`과 `reconnect_stop.log`에 근거를 보존했다. 런타임 코드 추가 수정 없이 통과했으며, 인증은 이미 성공한 operator로 모형화했다. 소켓·ROS·실제 자식·실물 출력은 실행하지 않았다.
