# 평가용 초기 차량·paused 시작 검증

2026-10-11. GPT 주 에이전트 설계·구현·분석, GPT-6 Luna 실행·대기·결과 회수. 기본 경로·독립 보호층·차량 물성은 유지한다.

## 실행 전 등록

- `evaluation_startup:=service`가 기존 기본값이다. 선택형 `embedded_paused`는 생성된 실행 사본 월드에 같은 차량 모델·슬롯 pose(z=0.01)를 미리 포함하고 `gz sim`의 `-r`를 생략한다. 공식/파생 원본 파일은 바꾸지 않는다.
- 평가기는 sim 0·paused stats 2개 이상, ROS publisher/보호 경로/arm·stop 서비스, gz 추적/접촉 publisher 및 subscriber 발견을 요구한 뒤 해당 시험 파티션의 world control로 unpause한다. 구독 발견은 실제 수신 보장이 아니므로 첫 물리 갱신·추적 연속성·센서/입력·STOP는 종료 후 따로 감사한다.
- 초기 준비 한도 90초, 기존 전체 600초·보호 벽시계 timeout은 그대로다. 준비 전에 시계나 물리 추적이 진행하면 거부한다. 실제 watchdog/같은 stamp 정책/명령 bridge는 수정하지 않는다.
- [Gazebo Harmonic 공식 pause/run API](https://gazebosim.org/api/sim/8/pause_run_simulation.html)를 사용한다. Python gz transport 바인딩이 설치되지 않아 기존 gz CLI를 사용하며 새 패키지는 설치하지 않는다.

최대 신규 Gazebo **3회**: S1 service, E1 embedded_paused, E2 embedded_paused. 모든 사례는 기존 공식 분할 노면·1 ms·passive_damping·10/8 고정 명령·sim_grid 위상 10 ms·GC defer·명령 추적을 동일하게 사용한다. 각 실행 뒤 실패 시 자동 재시도하지 않는다. 시작 관문 구현 오류가 있으면 실패 증거를 보존한 뒤 남은 등록 횟수에서 수정 검증 여부를 판단한다. 추가 반복·고속·G1/G2로 확대하지 않는다.

수용 조건:

1. 기존 입력 유효·명시 STOP·차체/바퀴 실제 정지·정상 종료·소스/원시 증거 검사 통과.
2. embedded는 첫 물리 갱신 `iteration=1, sim_ns=1000000, dt_ns=1000000`, trace 순번 1부터 연속, 해제 전 sim 0/paused 관측. 두 반복에서 초기 갱신 값 동일.
3. 삽입한 모델에서 최상위 pose만 제거하면 `runtime_vehicle.sdf` 모델과 같고, 그 모델을 제거한 실행 월드는 service 실행 월드와 구조상 같아야 한다(임시 메시 경로만 정규화). 기본 월드/차량·플러그인·센서·MCU/보호 소스 보존.
4. 명령 적용 차이·초기 갱신 이후 물리 결과 차이는 남아도 그대로 보고한다. 초기 생성 시점 고정과 전체 실행의 결정성은 다르다. 성공할 때까지 물리/보호 임계값을 바꾸지 않는다.

커밋/푸시·외부 게시·Jetson·전원 변경은 하지 않는다. 이전 GC 대조와 실패 증거는 그대로 보존한다.

## 결과

| 실행 | 계약·방식 | 첫 물리 갱신 | 요청 최대 지연 | 입력·명시 STOP·실제 정지·종료 |
|---|---|---|---|---|
| S1 | r5 / service | iteration 2, 2 ms | 0 ms | 모두 통과 |
| E1 | r5 / embedded_paused | 시작하지 않음 | 요청 없음 | 준비 검사 90초 timeout. 정상 종료만 통과 |
| E2 | r6 / embedded_paused | iteration 1, 1 ms | 0 ms | 모두 통과 |

E1은 contact publisher 수를 1개로 잘못 가정한 준비 검사 오류다. [수정 등록](ADDENDUM.md)에 따라 네 바퀴 센서=정확히 4개로 수정했다. E1을 주행/정지 시험 성공으로 계산하지 않는다.

E2는 약 11.90초(벽시계) 뒤 준비를 확인하고 unpause했다. 해제 전 sim 0/paused, 첫 추적부터 순번 연속, 1 ms 물리 step, 원래 GC 상태 복원, 잔존 프로세스 0을 확인했다. 기본 service 모델과 비교해 삽입 모델의 최상위 pose를 제외한 차량, 차량을 제외한 월드, 메시/프로필/슬롯이 같았다. 제어·센서·보호·MCU·구동 플러그인은 바꾸지 않았다.

**수정본 단회 기능 확인만 통과했다.** 사전 3회 상한 때문에 수정본 두 반복 조건은 미충족이며 `planned_repetition_acceptance=false`로 남긴다. 명령 시작 시각은 S1/E2 모두 2.01 s였지만 차량 삽입 시점이 다르므로 물리 결과나 명령 적용 결정성을 비교하지 않는다. 기본값은 service다.

검사 중 ROS Python 환경이 빠진 셸 호출에서 xacro 수집 오류가 발생했고, 환경을 포함한 run.sh로 다시 수행하여 관련 141개 검사를 통과했다. 최초 전체 회귀 1,365개 통과/2개 실패는 두 과거 검사에서 r2 launch 해시를 직접 참조한 문제였다. 기존 R0 파일 집합/불변 조건과 원본 의미를 유지하며 r6 계약까지 연결했다. 최초 로그를 덮어쓰지 않는다.

최종 전체 회귀는 **1,367개 통과**했다. 이전 실패 로그는 그대로 보존했다.

근거: [전체 실행·실패 감사](analysis_all.json), `status_*.json`, 각 실행의 `source_snapshot.zip`·report·원시 해시, [마감 감사](final_manifest.json). `pytest_finaltests.log`는 최초 실패, `pytest_finaltests_v2.log`는 후속 전체 회귀 결과다.
