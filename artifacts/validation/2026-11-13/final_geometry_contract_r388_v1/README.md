# r388 최종 차량 기하 전달 계약 — staged 준비

상태: **실행 전**. 공통 소스는 수정하지 않았고 WSL/Jetson/ROS/장치를 실행하지 않았다. `staged/`와 `geometry_contract.patch`는 통합 검토용이며, 기본 채택/새 주행 승인을 뜻하지 않는다. r386 거리 예산 후보는 계속 미실행 보류하며 이 묶음에 포함하지 않는다.

## 확인한 불일치와 범위

최종 CAD YAML은 이미 올바른 값을 갖는다. `vehicle.final_cad_2026_11_13.yaml`은 계획 외곽207×150mm, 축거128mm, 윤거123.9mm, 바퀴 반경32.5mm, 바퀴 폭26.1mm다. 물리 모델은 이 명시값을 받았으나, `FollowConfig`에서 윤거/바퀴 반경/폭을 전달하지 않아 현재 검사와 계획의 `LateralIntentConfig`는 윤거135mm·반경32.5mm·폭12mm 기본값을 사용했다.

| 전달 경로 | 원 상태 | 준비한 처리 |
|---|---|---|
| 선택 vehicle YAML → simulation xacro | 축거·윤거·반경·폭 전달됨 | 변경 없음 |
| 선택 YAML → launch planning parameters | 길이·폭만 전달 | 명시 윤거·반경·폭 추가, 기존 축거 전달 유지 |
| ROS → LocalPursuit → FollowConfig | 새3치수 없음 | 새3매개변수 선언/전달 |
| FollowConfig → pass_release / M3Execution / M3Observed | 새3치수 누락 | 세 생성 경로에서 같은 필드 전달 |
| 현재 rebase → pass_release.lateral_config | 잘못 남은 기본값 사용 | 위 공통 전달로 수정; wrapper/보호식 수정 없음 |
| trace/별도 계산 프로세스 → FollowConfig | `asdict`/키워드 재구성 | 새 명시 필드는 그대로 기록·전달, 누락된 옛 trace는 legacy 기본값 |
| 평가 `validate_g1_traffic` → `g1_traffic_eval` | 선택 YAML의 외곽·축거·윤거 전달됨 | 변경 없음. 바퀴 중심 노면 지표를 타이어 전체 접촉 평가로 확대하지 않음 |
| 독립 LiDAR 보호 | 선택 외곽·축거 기반 swept 검사 | 이 작업의 보호층/10mm/TTL/물성 변경 없음. 해당 swept 함수는 타이어3치수 인자를 쓰지 않음 |
| hardware preview | 기존 preview YAML만 사용 | 명시 `--vehicle-config`일 때만 기하 덮어쓰기; 생략 시 기존 설정 |

윤거는11.1mm 작아지지만 타이어 폭은14.1mm 넓어진다. 직진 타이어 전체 폭은147→150mm다. 회전한 바퀴 모서리, 원호, 현재 차체 밖 질의는 서로 다르게 변할 수 있어 통과/거절 어느 방향도 미리 단정하지 않는다. r377의10.766mm 초과를 없애기 위한 차량 축소가 아니다.

## staged 파일9개

- `src/arena_autonomy/arena_autonomy/local_follow.py`: 새3필드와 유한 양수 검증. 누락 기본값은 당시 `.135/.0325/.012` 유지.
- `src/arena_autonomy/arena_autonomy/local_pursuit.py`: ROS에서 새3필드 전달.
- `src/arena_autonomy/arena_autonomy/pass_release_trial.py`: 측방 설정 생성 때 전달.
- `src/arena_autonomy/arena_autonomy/m3_execution_trial.py`: M3 설정 생성 때 전달.
- `src/arena_autonomy/arena_autonomy/m3_observed_follow.py`: M3 설정 생성 때 전달.
- `src/arena_autonomy/arena_autonomy/vehicle_geometry.py`: 새 순수 YAML→매개변수 변환. 명시 YAML의 누락/비수치/비유한/음수/외곽 미수용은 거절한다.
- `src/arena_bringup/launch/simulation.launch.py`: 선택 YAML의 새3치수 전달. 기존 중앙 등가 조향 변환식을 순수 공유함수로 이동.
- `scripts/probe_g1_hardware_shadow.py`: 선택 YAML 검증·해시·ROS 기하 전달·사용 설정 기록.
- `scripts/run_hardware_sensor_preview.py`: 선택 옵션 전달 및 장치 확인/자식 실행 전 검증.

현재 공통 원문8개는 `base/`에 바이트 그대로 보존했다. `stage_manifest.json`에는 변경 전/후 해시가 있다. 최종 CAD YAML과 기본 preview YAML은 수정하지 않는다. 명시 YAML의 잘못된 값은 legacy값으로 조용히 대체하지 않는다. 순수 회귀는 예전/공유 중앙 등가각이 유효한 ideal/CAD 설정에서 정확히 같은지 비교한다.

하드웨어 preview가 명시값을 선택하면 길이/폭/윤거/반경/바퀴 폭, 축거, 후륜축 x=`-wheelbase/2`, motion 반경, 기존식의 중앙 등가 조향한계만 전달한다. 속도·기어비·센서 시간 정책·구동 출력·장치 실행 정책은 그대로다. 실제 장치 연결/출력 시험은 이 묶음에 없다.

## 마지막 입력 비교

원 r377 `sources.zip`(1350개 보존)과 `follow_decisions.jsonl`을 읽어 마지막 현재 판단 index222/전체223, reference30.022s의 **원 header와 원 call 행 바이트**만 별도 보존한다. 전체 trace 해시/행 번호/개별 행 해시는 `input_manifest.json`에 기록한다. 평가 정답은 입력에 넣지 않는다.

`build_bundle.py`는 r377 동결 소스를 `frozen/`에 복사한다. 두 번째 `geometry/`에도 같은 원문을 넣고, 현재 `base→staged`의 정확히 일치하는 문맥 hunk만 이관한다. 현재 파일 전체로 덮어쓰지 않아 r379 등 다른 변경이 섞이지 않는다. 다른 날짜의 r281/r266 후보를 포함한 r375 계보 전체는 원문 그대로다. 허용9파일 외 두 arm의 차이가 있으면 번들 준비를 거절한다.

세 별도 프로세스에서 다음을 계산한다.

1. `frozen`: 당시 r377 소스+원 설정으로 원 최종 출력 재현.
2. `legacy`: 기하 전달 staged+원 설정(새필드 없는 header의 옛 기본값). frozen 출력과 질의 배열의 **완전 동일**을 확인.
3. `explicit`: 같은 staged+확정 YAML3치수. 길이·폭·축거·중앙 조향한계가 원 입력과 같은지 먼저 확인하며, 출력/관측/시간/이력/원 선택 경로는 바꾸지 않음.

각 arm은 r375 wrapper의 전체 현재 결과를 새로 계산한다. `.45/.15/.10` 원 호출 경로를 그대로 두며 r386이나 새 속도 재시도를 넣지 않는다. 저장된 기대값과 같은지 기존 `equal_output`(기존1e-10)와 정확한 트리 비교를 **별도로** 남긴다. 새 tolerance나 삭제 필드는 없다. 원 재현 exact 또는 legacy 불변 실패 시 비교 `passed=false`이며 전체 diff를 보존한다. 명시 기하 arm의 속도/거절 변화 자체는 성공/실패 필터가 아니며 보고한다.

`motion_queries` 반환 배열만 복사하는 계측을 감싼다. 제어 호출이 끝난 뒤 실제 질의 좌표의 exact 공통/추가/제거 집합과 unknown/masked 부분의 개수·x/y 범위·차체 옆 초과 범위를 비교한다. 원호 거리·현재/이전 조향·목표속도가 같은 호출끼리만 짝지으며, 새 기하로 더 이상 도달하지 않거나 추가된 호출은 따로 표시한다. 전체 배열은 `queries.npz` 로컬 산출물, 전체 진단은 arm별 `report.json`, 비교 요약은 `comparison.json`이다. 이 계측의 벽시간은 비용/명령 만료/10Hz 근거로 사용하지 않는다.

## 실행 인계 및 종료 기준

`execute_isolated.py`는 지정 WSL x86 담당자가 새 `$HOME/it-arena-r388-*`에만 풀어 실행한다. 동결 원소스 native 빌드1회(120s), 동일 바이너리를 양 arm에 복사/해시 확인, 순수 회귀1회(120s), 세 arm 각1회(각120s), 요약1회(120s)다. ROS/시뮬/실물 장치는 열지 않는다. 실패 시 이후 단계 중단, 보유 자식만 종료, 로그/종료코드/소스 해시를 결과 ZIP에 보존한다. 기존 루트/결과/번들을 덮지 않는다.

고정 archive/bootstrap SHA와 명령은 `HANDOFF.md`, 원문 소스 목록은 `source_manifest.json`, 준비 상태는 `bundle_receipt.json`에 남긴다. 실행 전 준비 검사는 Python AST·원본 바이트/해시·patch 문맥/번들 무결성뿐이다. 테스트와 마지막 입력 계산은 아직 실행하지 않았다.

결과의 범위는 **마지막 저장 현재 입력1개**다. 새 기하를 처음부터 사용했을 때 계획·이력·실제 주행이 어떻게 변하는지는 증명하지 않는다. 정체 해소/전체랩/실차 안전·연속속도 달성으로 확대하지 않으며, 대표 입력 확대와 r386 재개는 이 원인 분리 결과를 보고 별도 판단한다.
