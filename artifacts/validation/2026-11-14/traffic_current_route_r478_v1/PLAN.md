# r478 현재 경로 우선 변경 후 실제 점유 교통 대표

상태: **Windows 준비 완료, 미실행. root 검토 전 실행 금지.** WSL은 r477 실행 담당이 독점한다. 이 준비에서는 WSL/Jetson 접근·빌드·ROS/Gazebo 실행·공통 소스 수정을 하지 않았다.

## 선택

`straight_moving_lead`, seed 0, GREEN 이후 **10 sim s, 한 번**을 선택한다. 원본은 [새 코스 등록](../../2026-11-07/traffic_course_migration_v1/new_track_course.json)의 같은 case다. 현재 계획의 ‘단독 → 출발 변형 → 기존 앞차·옆차 대표’ 순서를 따르며, 빠른 자차가 느린 앞차에 접근하고 앞차가 가속할 때 측방 접근/추월/뒤 합류 가능성이 생기는 짧은 장면이다. r474가 실제 점유 대응을 보존하면서 불필요 정체를 줄이는지를 확인한다.

단순 중앙 0.30 m/s 추종 14초보다 측방 상황을 포함할 여지가 있고, 60초·두 상대 전체랩보다 범위가 작다. 11/2 `restart_during_bypass`는 옛 코스의 첫 코너 좌표를 사용하므로 이번 새 코스에 조용히 이식하지 않았다.

| 등록 항목 | 고정값 |
| --- | --- |
| 트랙/출발 | `official_v2026_11_06`, 기존 grid slot 3, nominal 위치/방향 |
| 상대 수 | 1, `eval_g1b_actor` |
| 초기 종방향 중심 간격 | 1.60 m, seed offset 0 |
| 상대 횡방향 offset | −0.12 m, 진행 방향의 오른쪽 |
| 상대 속도 | 0–4 s: 0.30 m/s; 4–4.2 s: 0.40; 4.2–4.4 s: 0.50; 4.4–10 s: 0.58 |
| 상대 각본 이동량 | 4.628 m; 원 각본/시각 그대로 |
| 상대 형상 | 200×150×220 mm, `recessed_sides` 시각 형상, 접촉/간격은 200×150 mm 외접 box |
| 상대 갱신 | command 100 Hz, pose publisher 100 Hz; r475 actor 템플릿 그대로 |
| 자차 | r475 확정 CAD 차량·센서 YAML 그대로, 같은 `g1_open_1p5` 속도와 native/process 실행 |
| 시설/신호 | r475 old610 시설 3개·공식 RGB 출발·ID20/30 blank 동일 |
| 물리/센서 | r475 RTF 0.25·동일 timestep/물성·native C1 입력; 추가 잡음/지연 주입 없음 |
| 마커 observer | OFF |

현재 C1 높이는 134.6 mm다. r6 원 상대 높이 120 mm를 유지하면 LiDAR보다 낮아져 의도한 점유 검사가 되지 않는다. 따라서 root가 승인한 현재 config의 220 mm 상대 가정을 사용한다. 이는 **LiDAR에 보이는 상대**에 관한 새 장면이며, 낮은 차량·희소 상부 감지 또는 마커 보조는 확인하지 않는다. 원 r6와 동일 조건 개선율도 주장하지 않는다.

## 보존·변경 범위

- 원 r6는 2.77 m 진행·최장저속 4.249 s, 원 판정 및 보충 판정 실패다. [당시 결과](../../2026-11-07/traffic_course_migration_v1/r6/straight_moving_lead/assessment_run2.json)를 수정하거나 현재 성공으로 소급하지 않는다.
- [originals.zip](originals.zip)에 원 각본·평가기·r6 등록·원 보고서·당시 source snapshot과 현 공통 traffic runner/r475 연결 근거를 보존한다. [provenance.json](provenance.json)에 원 해시와 복사 관계를 기록한다.
- 원 case/schedule/10초/횡offset/seed·1.6 m 시작 간격은 그대로다. 최신 자차·센서·정책·시설·actor 높이/시각 형상/갱신율을 명시적으로 이식한다.
- 공통 `scripts/validate_g1_traffic.py`를 그대로 호출한다. r475 launch 4파일·선택 adapter·시설 SDF를 그대로 복사하고, 평가 actor SDF에 시설 3개를 추가한다. 원 actor/contact/정답 자세 구독은 유지한다.
- `/drive`→독립 보호층→`/drive/safe`, 기존 STOP·정상 종료·기록 계약을 유지한다. actor 정답은 평가/각본에만 쓰인다.

## 판정·회수

10초 대표 구간이며 완주 시험이 아니다. 특정 최종 mode, 강제 추월 진입, 반드시 RELEASED 전환을 성공 조건으로 만들지 않는다. 실제로 측방 단계까지 도달하지 않았다면 전방 대표만 성립한 것으로 범위를 제한한다.

원 `report.json`의 `passed`, 모든 간격·종점 간격·뒤 유지·노면 기준은 그대로 보존한다. 추월로 원 ‘항상 뒤’ 기준이 실패해도 원 실패를 지우지 않는다. 실제 표본/스윕 간격 하한과 접촉 수치를 함께 보고한다. 별도 새 최소 간격 임계값을 추가하지 않는다.

주 판단은 기존 `straight_moving_lead`의 진행 ≥2.5 m·이동 앞차 중 최장정지 ≤2.5 s, 실제 접촉/겹침, 현재 자차 기하의 노면, STOP·정상 종료·입력 불변이다. 기존 r475와 같은 wheel-center 평가로 최대 2개 노면 밖 여부를 보고한다. 원 r6의 강제 특정 모드/진입 시각 기준은 이번 목표가 아니다. actor 각본/입력 유효성은 기존 traffic evaluator가 판정한다.

기존 관찰 도구의 8 sim s 무진행 또는 벽 접촉 STOP을 재사용한다. 10초 창이 정상 종료되면 기존 평가기가 명시 STOP과 자식 회수를 수행한다. 실제 actor 접촉/겹침을 확인하면 추가 진행하지 않고 같은 소유 STOP을 사용한다. STOP 요청·MCU latch·실제 정착·종료는 구분해 보존한다. 실패/미성립이어도 자동 반복하지 않는다.

## 고정 명령

현재는 Windows 정적 준비까지만 수행한다. 아래 freeze와 WSL 명령은 **r477 회수·자원 인계 및 root 검토 후** 한 번 수행한다. common이 r475 동결 해시와 다르면 변경 목록을 root에 넘기며 조용히 통과시키지 않는다. r475의 빌드/native10/import 증거를 재사용하고 별도 빌드는 예약하지 않는다.

```powershell
# Windows, root 검토 후 최종 입력 동결
C:/Python314/python.exe -B artifacts/validation/2026-11-14/traffic_current_route_r478_v1/prepare.py --freeze
```

root는 최종 `prepared.json`/`sources.zip` SHA를 확인해 `root_execution_review.json`에 `root_reviewed=true`, `execution_limit=1`, `repeat_on_failure=false`, `prepared_sha256`, `source_archive_sha256`를 기록한다.

```powershell
# 인계된 WSL에서 단독 1회. execute_once.sh → python3 run.py
wsl.exe -d Ubuntu-24.04 -- bash "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_route_r478_v1/execution_v1/execute_once.sh"
```

전체 CLI는 [arguments.json](arguments.json)에 고정한다. root review/동결/한 번 실행 검사를 통과하기 전에는 `run.py`가 실행하지 않는다. Windows에서 백그라운드 실행 시 기존 r475처럼 `Start-Process -WindowStyle Hidden`으로 실행하고 stdout/stderr·Windows PID/종료 코드를 `execution_v1`에 보존한다.

```powershell
# Windows에서 기존 로그 관찰
C:/Python314/python.exe -B artifacts/validation/2026-11-14/traffic_current_route_r478_v1/execution_v1/observe.py
# 필요한 경우, 소유 시험에만 STOP; 실행 중일 때 한 번
wsl.exe -d Ubuntu-24.04 -- bash "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_route_r478_v1/execution_v1/stop_once.sh"
# 실행이 끝난 후 소유 PID/partition 회수 확인
wsl.exe -d Ubuntu-24.04 -- python3 "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_route_r478_v1/execution_v1/check_owned_processes.py"
```

후처리 `collect_result.py`는 기존 r475 환경의 scripts/src Python 경로에서 실행한다. 현재 Windows에는 Shapely/PyYAML이 없어 공통 평가기 import·기하 검사 실행은 하지 않았다. 패키지를 설치하거나 WSL에 접근하지 않았다. 구문·원 각본 동일·CLI 옵션·actor/시설 XML 연결만 정적으로 확인하며, 실제 actor 경로 검사는 기존 평가기의 실행 전 검사에 남긴다.
