# r361 계획 저속 후보의 확정 기하 제한 주행 준비 — r364

**현재는 준비만 완료했다. 실행·WSL·빌드·노드 시험은 하지 않았다.**
실행 담당은 소유권 인계와 공통 소스 동결 검토를 마친 뒤 아래 노드 시험/제한1회를 수행한다.
r353 원 실행과 실패는 보존하며 이번 폴더에 결과를 새로 기록한다.

## 동결 조건

| 항목 | 유지값 |
|---|---|
| 차량 | 축거128mm·윤거123.9mm·타이어 폭26.1mm/공칭65mm, 실제 차체199.7mm/X+3.65mm, 계획207×150mm |
| 물성/구동 | r353의 질량2kg 근사·접지·조향 한계·모터축28count·2.5×81/17 그대로 |
| 센서 | r347 확정 배치·전방 배선6mm/움직임3mm, native C1 10Hz, ID49 Z140mm |
| 시설/노면 | 구형610 신호/지주, ID20/30 누락, `road_level` |
| 교통/시간 | 높이220mm 상대2대, 원 속도/각본, 최대80sim초, RTF0.25, 지연 주입0 |
| 실행 경계 | r336 발행/두 슬롯·r352 현재 검사·독립 보호·STOP/정상종료/원 평가 판정 그대로 |

**유일한 행동 변경:** `low_solo_worker.worker(stage='plan')`에서 기존
`boundary.install()` 뒤 [`r361 candidate.enabled()`](../released_crawl_r361_v1/candidate.py)를
열고, 그 context 안에서 기존 `prior.ORIGINAL(stage, *args)`를 호출한다.
current 자식은 기존 r352 전체 현재 검사를 그대로 사용한다. 카메라60fps/Jetson 후속 비교는 이 시험에 섞지 않는다.

r361은 현재 RELEASED·현재 양수 G1 경로/fully_observed·원 측정저속에서 기존
0.35/0.25/0.15m/s 실패 뒤0.10m/s를 같은 실제속도/두 조향/시각으로 한 번 검사한다.
unknown·10mm 꼬리·관측/시간·독립 보호·정지 경계를 완화하지 않는다.

## 연결과 증거

- `launch_prefix.py` → 이 폴더의 `course.launch.py`/`node_entry.py`/`profile.json`.
- `node_entry.install` → 기존 r336 설치 후 자신의 `low_solo_worker.install` 연결.
  모듈 경로를 확인하여 이전 r353 worker가 import cache에서 재사용되면 거절한다.
- parent trace의 `released_plan_crawl_policy`는 r361 hash·plan-only·기본OFF·r352 유지와
  실제 자식 설치 증거 경로를 기록한다. 이 이벤트만으로 자식 설치를 입증하지 않는다.
- 각 선택형 plan 자식은 `worker_policy_receipts/plan_<pid>_<ns>_installed.json`에
  r361 selector/raw-speed 경계의 실제 설치 여부와 hash를 기록한 뒤 원 worker를 실행한다.
  종료 receipt는 context 복구와 원 worker 반환 여부를 기록한다. 주행 성공 판정은 아니다.
- `test_node.py`는 **자신의 node_entry를 절대 파일 경로로 import**하고, parent policy
  이벤트·실제 plan PID receipt·r352 유지·STOP/RESET/두 자식 정상종료를 검사한다.
  시험 receipt는 `tmp_path`로 보내 실제 주행 폴더에 섞지 않는다.

## 실행 전 parent 검토

1. r362를 포함한 공통 src/테스트 수정이 끝나고 WSL/ROS/Gazebo의 실행 소유자가 한 명인지 확인한다.
   준비 시각의 AST/hash 기록은 실행 동결이 아니다. r362 제품 어댑터를 이번 실행의
   r336/r352 연결로 교체하지 않으며 동시 편집도 하지 않는다.
2. `static_audit.json`, 이번 diff, r361 후보 해시와 저장 검증을 검토한다.
   패키지/설치본과 현재 소스의 일치 여부 및 필요한 빌드는 실행 담당이 판단한다.
3. 아래 노드 시험을 독립 도메인에서 수행하고 receipt/STOP/종료를 확인한다.
   아래181은 이 시험을 위한 제안 도메인으로, 실행 담당이 비점유를 확인한 뒤 사용한다.
4. `run1/`, `registered.json`, `sources.zip`이 없는 새 상태에서 제한1회를 실행한다.
   `run.py`가 실행 직전 전체 src·scripts·tests와 r361/r352 및 관련 시험 소스를 archive에 넣고
   해시를 등록한다. 이후 종료까지 해당 입력을 수정하지 않는다.

## 후속 실행 담당용 정확한 명령 — 이번 준비에서는 미실행

저장소 루트의 Windows PowerShell에서 노드 연결 검사:

```powershell
python build/wsl_namespace_exec.py "cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS='' PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:$PYTHONPATH python3 -m pytest -q --tb=short -p no:cacheprovider artifacts/validation/2026-11-13/released_crawl_drive_r364_v1/test_node.py"
```

동결 검토 후 제한1회:

```powershell
python build/wsl_namespace_exec.py "cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONDONTWRITEBYTECODE=1 python3 artifacts/validation/2026-11-13/released_crawl_drive_r364_v1/run.py"
```

지속 무진행/위험 관측 때 소유 시험에 조기 STOP 요청:

```powershell
python build/wsl_namespace_exec.py "cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONDONTWRITEBYTECODE=1 python3 artifacts/validation/2026-11-13/released_crawl_drive_r364_v1/stop_owned.py"
```

`stop_owned.py`는 정확한 소유 PID/작업 폴더를 검사한다. 위 run 명령의
`python3 <상대경로>/run.py` 형태를 유지한다(`-u` 또는 다른 argv형태로 바꾸지 않는다).
STOP 뒤 평가기의 원 STOP검증·정상종료·원 실패 판정을 보존한다.

## 회수할 결과

원 r35318.58m 이후 정체 구간 통과 여부, 실제 후보 발동/현재검사/독립보호 결과,
진행·전체 랩·긴 저속/재정체·접촉/노면·STOP·정상종료를 함께 본다.
기하 runtime audit, parent policy hash와 모든 plan PID receipt, 원 입력/ZIP 불변을 확인한다.
새 실패는 새 원인과 함께 보존하며 무변경 재실행하지 않는다. 제한1회 성공도 전체 교통/실차 검증으로 확대하지 않는다.


실행 환경 보완: 첫 노드 검사는 예시의 `PYTHONPATH`가 ROS setup 경로를 덮어써 rclpy를 불러오지 못했고 skip이었다(`node_tests.log`). 기존 ROS 경로를 뒤에 보존한 bash 실행(`build/r364_node_test.sh`)으로 수정해 1검사 통과(`node_tests_retry.log`)를 확인한 뒤에만 주행으로 넘어간다. PowerShell에서 이 예시를 직접 붙여넣기보다 bash 파일 안에서 `$PYTHONPATH`를 확장해야 한다. 코드 실패/주행 실패로 혼합하거나 원 skip을 삭제하지 않는다.
