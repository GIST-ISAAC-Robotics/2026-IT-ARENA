# r499 기존 오른쪽 옆차 통과·복귀 대표 — 실행 전 준비

선정 케이스는 11/8 `right_side_newcourse_v1`의 **`right_side_start` 1개**입니다. 자차 앞 0.50 m·오른쪽 0.23 m에서 0.50 m/s로 이동하는 actor 옆을 자차가 지나고, 이후 주행·복귀를 이어가는 **12 sim초** 대표입니다. 빠른 뒤차에 추월당하는 장면은 아닙니다. 폴더명에는 먼저 검토했다가 시설 관통으로 거절한 피추월 후보의 이름이 남아 있으며, 실제 선택은 `config.json`과 `arguments.json`의 `right_side_start`입니다.

단독 한 랩 r492와 앞차 추종 대표 r496 이후, 현재 경로 중심 정책이 측방 상대의 실제 점유를 유지하면서 통과 후 정체 없이 진행하는지를 확인할 작은 기존 장면입니다. 11/8 원 실행은 실제 나란히 1.010초·통과 이후 진행이 성립했지만 case별 acceptance 별칭 누락으로 원 평가 실행이 실패했습니다. 당시 원본과 사후 재평가를 보존하고, 이번 새 구성에는 기존 `slow_lead`와 정확히 같은 수치의 `right_side_start` 별칭을 **실행 전에** 넣습니다. 과거 오류/실패를 성공으로 소급하지 않습니다.

## 현재 입력·변경 범위

- 자차·센서·정책·속도는 r496 최신 공통과 동일합니다. 확정 CAD 계획 외형 207×150 mm, 축거128 mm·윤거123.9 mm, 전방배선6 mm C1 Z134.6 mm, B0390/IMU, `g1_open_1p5`/`medium_v1`, 마커 보조 OFF, `/drive`→독립 guard→`/drive/safe`를 유지합니다. r498은 stage 상태라 포함하지 않습니다.
- 시설 3개·현행 코스·원 출발슬롯3·road_level·RTF0.25·STOP/종료는 r496과 같습니다. 지도·상대 일정·정답 pose는 평가기 전용입니다.
- actor의 원 평면 외형200×150 mm·상자 형상·50 Hz 명령/100 Hz pose·+.50 m/−.23 m·.50 m/s·12초를 유지합니다. 원 높이120 mm는 현재 C1 높이 아래이므로 **220 mm의 보이는 상대 행동 가정**으로 바꿉니다. 원120 mm·낮은 차량·희소 상부 감지 성공으로 해석하지 않습니다. r496 actor의 `recessed_sides`와 구별해 이 원 대표의 기본 `box`를 명시합니다.
- 공통 코드 편집·새 빌드·장치 접근은 없습니다. r496의 실제 공통 native6파일166검사는 같은 소스/바이너리 SHA를 확인해 재사용하며 반복하지 않습니다.

## 배치 유효성과 거절한 원 후보

현재 코스에서 **actor 전체 6.0 m 경로 601표본(최대1 cm)**을 기존 도로/벽 검사와 시설 mesh의 **0–220 mm 전체 높이 투영**으로 대조했습니다. 초기 자차 간격 **0.296500 m**, 초기 겹침0, actor차체 도로 밖 면적0, 시설 관통0, 최소 시설 간격 **0.047500 m**로 통과했습니다. 실제 나란히·추월·복귀 성립은 폐루프에서 따로 확인해야 합니다.

먼저 선택한 r24 `left_rear_overtakes`(뒤−1.10 m·왼쪽+.28 m·6.8초)는 같은 시설에서 **신호 지주와25표본 겹침**이 나와 거절했습니다. 968표본·최대 겹침면적0.0001250004 m², 원 `geometry.json`/`geometry_rejected_rear_source.py`를 보존합니다. 시설을 빼거나 .28 m를 조금 줄여 같은 장면이라고 재사용하지 않았습니다. 대체 후보는 별개의 이미 존재하는 `right_side_start`입니다.

정적 계산은 Windows 두 Python에 Shapely가 없어 설치하지 않고, 배정된 WSL 기존 Python으로 원후보1회/대체1회를 수행했습니다. ROS 초기화·시뮬레이터·빌드는 없었고 두 계산이 끝난 뒤 WSL을 반환했습니다. 첫 import 오류는 `geometry_attempt1.txt`/`geometry_attempt2.txt`에 남겼습니다.

## 판정과 회수

원 평가의 **150 mm 간격·앞차 뒤 유지·최종 간격·차체 도로 안**과 기존 별도 목표의 **80 mm 간격·최대2바퀴·2.5초 저속 한계·마지막2초0.5 m 진행·복귀/기존 FOLLOW 조건**을 분리해 모두 기록합니다. 옆차를 앞지르는 대표에서 원 '계속 앞차 뒤 유지'가 실패하더라도 원 FAIL을 변경하지 않습니다. 별도 목표의 숫자/상태 조건도 낮추지 않습니다. 기존 바퀴 `any`/`center`/`full` 해석을 현재 CAD 치수로 계산하고, 공식 두 바퀴 잔디 허용과 보수적인 차체 도로 안 판정은 구분합니다. 바퀴 일부 이탈 정의·긴 잔디 주행은 심판 해석 한계가 남으며 계산상 바퀴 지표를 실제 접촉/공식 판정으로 부르지 않습니다.

기존 8 sim초 무진행·벽접촉 관측이면 소유 STOP을 요청합니다. 원 STOP 6 sim초 이내·0.5초 안정·명시 MCU STOP·14자식 종료·소스/ZIP 불변·PID 회수를 확인합니다. 한 번 실행 후 실패여도 자동 반복하지 않습니다. 사용자가 차량 준비를 알리면 새 주행을 시작하지 않고 실물 우선으로 전환합니다.

## 시간·고정 명령

직전 r496의 GREEN10초 시험은 준비/신호/STOP을 포함해 실제 wall226.59초였습니다. 이번12초는 같은 실행 부하에서 **약4–6분**을 예상합니다. 이는 추정이며 RTF0.25 설정이 실제 처리율을 보장하지 않습니다. 기존 startup150초·전체wall1500초·STOPwall120초 한계는 그대로입니다.

root가 조건을 검토하고 WSL 소유권·`source_selection.json` 동결 승인을 준 뒤 기존 preflight/import→freeze 명령:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/rear_overtake_current_geometry_r499_v1/prepare_once.sh'
```

그 동결의 prepared/ZIP SHA를 root가 별도 검토해 `root_execution_review.json`에 기록한 뒤 **단1회**:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/rear_overtake_current_geometry_r499_v1/execution_v1/execute_once.sh'
```

감시는 기존 `execution_v1/observe.py`, 필요시 소유 STOP은 `execution_v1/stop_once.sh`, 종료 후 원 판정/별도 목표/현재 바퀴 기하/소스/PID 회수는 `execution_v1/finish_collection.bash`를 씁니다. 아직 소스 동결·등록·실제 주행은 **0회**입니다.

근거: [원 사례](../../2026-11-08/right_side_newcourse_v1/README.md), [선정 경로 정적 결과](geometry_right_side.json), [거절한 피추월 경로](geometry.json), [복사/변경 출처](recipe_provenance.json), [고정 인자](arguments.json), [동결 전 선택 상태](source_selection.json).
