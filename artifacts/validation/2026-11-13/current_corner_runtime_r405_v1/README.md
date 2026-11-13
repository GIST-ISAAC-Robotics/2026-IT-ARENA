# r405 현재 코너 거리 예산의 선택형 패키지 연결 — 준비

2026-11-13. 공통 소스를 수정하지 않고 staged8파일, 제한 회귀, 저장223개 대조 하네스를 준비했다. 아직 후보/pytest/WSL/ROS 노드/Gazebo/장치 실행은 하지 않았다. Windows에서 원문 대응·AST·정적 import 폐포·입력 추출·해시를 확인한 단계다. root 독립 검토와 지정 실행자의 검증 후에만 주행 등록을 판단한다. 기본 r377 및 실물 r401 선택은 그대로다.

근거는 r396 `full_current_v3`의 **실제 수정 후 판본 실행**이다. root가 감사한223 baseline exact, 양수119→213, 비대상129동일, 반례3조건 유지를 재현 대상으로 사용한다. 최초 동결 인계 위반과 당시 `98afd…`/실행 `eb3a…` 차이를 숨기지 않는다. `reference_sources.zip`에 원 감사 문서를 보존했으며 그 결과를 최초 판본의 무변경 실행으로 바꾸어 표기하지 않는다. 원 r386 pipeline의 실패도 그대로다.

## 구현과 불변 범위

| staged 파일 | 내용 |
|---|---|
| `race_runtime_policy/corner_budget.py` | r386 final `bundle_sources.zip`의 `r386/budget.py`와 바이트 동일. 거리식·1cm 질의·현재/이전 조향 합집합·측정속도 제동거리·반올림·상수 불변. |
| `race_runtime_policy/corner_candidate.py` | 원 candidate의 상대 import와 명시 `proof_callback` 인자만 연결. 과거 비교 callback이 무시하던 `relative_neighbor=True` 키워드는 이 호출에서 제거했다. eligibility·단일 제안·기존 core 최종 검사·region 및 원 identity 검사는 동일하다. |
| `race_runtime_policy/range_mask_component.py`, `profile_evidence.py` | r396 v3 staged 원문을 패키지 상대 import로 옮겼다. 부분 마스크/일반 무반환 성분, 전체 폭 합산, 원 출처/시각·점유 기준 불변. |
| `race_runtime_policy/evaluator.py` | 기존 released_relative baseline을1회 계산한 뒤 ON/eligible에서만 후보를 호출한다. 전역 함수 교체나 artifact 동적 import는 없다. 계획 단계는 기존 계산 그대로다. |
| `runtime_node_backend.py` | 선택/환경 검증과 ON에만 붙는 worker header를 준비한다. 기존 dual process/delivery·발행 ACK·원 시각 전달은 그대로다. |
| `g1_g2_node.py` | `g1_current_corner_budget='off'|'r396_v1'` 명시 옵션. worker 생성 전에 등록 backend와 simulation/time 조건을 확인한다. |
| `follow_trace.py` | root 승인 추가 파일. 선택형 keyword1개로 ON일 때만 trace header에 `current_corner_budget`를 쓴다. OFF의 header 키/순서/값과 다른 config는 유지한다. |

이전 wrapper를 또 호출하는 구조가 아니다. evaluator가 이미 계산한 baseline을 넘기고 후보는 eligible일 때 거리 제안 최대1회, 추가 core 최대1회, 최종 region 최대1회, component proof 최대1회만 수행한다. 원 baseline 내부의 기존 재검사와 component 내부의 body 선행 검사는 바꾸지 않았다. 그 내부 호출까지 모두1회라고 주장하지 않는다. 수락한 후보의 경로/조향/원 시각/두 조향 영역·보호 임계값을 바꾸지 않는다.

현재 공통 r403 frame 계약을 묶음에 포함했다. 예산 helper의 `source_valid`도 이 공통 함수를 사용한다. 이번 저장 집합은 모두 원 `laser_frame`/`simulation_acquisition`/`timing_verified=true`이므로 C1/host-estimated 적용을 동시에 시험하거나 승인하지 않는다. `source_ledger.json`의 원문/공통 이전/새 파일 해시와 `source_changes.patch`로 변경 범위를 검토한다. 원 common4파일은 `before_sources.zip`, 원 함수4개는 `reference_sources.zip`에 보존했다.

## 명시 선택 경계

ON은 기존 `g1_runtime_policy=cad_runtime_r377_v1`가 고른 native 두 슬롯 실행에 한정한다. 기존 등록은 process_candidate/fresh_native_v1/worker_event_v1/retained reference/native_v1/비-shadow 조건을 그대로 요구한다. 추가 ON 조건은 `use_sim_time is True`, `motion.verified is True`, `verified_only`이다. host_estimated_v1 및 실물 환경은 명시 거절한다. 별도 새 속도·TTL·점유·물성 조건은 추가하지 않는다.

노드, worker header, evaluator에서 선택을 검증한다. ON의 header에만 `current_corner_budget`와 simulation/verified/native2slot 환경 선언이 추가된다. evaluator는 각 입력의 timing policy와 true verified도 확인한다. OFF에는 새 header 키가 없으며 host-estimated 기존 r401 경로도 기존 header 그대로다. trace의 ON 표시는 요청한 설정의 기록이며 그 자체가 구동 허가가 아니다. `/drive`→독립 보호→`/drive/safe` 경계와 실제 발행/시간 만료 판정은 기존 경로의 책임으로 남는다.

## 제한 검증

새 `tests/test_current_corner_runtime.py`와 기존 `tests/test_runtime_node_backend.py`만 실행한다. 옵션 OFF/등록 이름/잘못된 환경·header·입력, OFF trace header 바이트, ON 기록, 실제 G1G2의 trace 생성 메서드를 NodeDouble로 호출하는 연결, 계획 출력 불변, 비대상 baseline1회 및 추가 후보 미호출, 명시 proof callback과 제안/core/region/proof 각1회 계약을 확인한다. 테스트에서 의존 함수를 가짜로 바꾸는 것은 호출 횟수 경계를 검사하기 위한 것이며 운영 소스에는 전역 monkeypatch가 없다. ROS 메시지 라이브러리는 import할 수 있지만 실제 노드·spin·발행은 하지 않는다.

`compare_current.py`는 패키지 ON evaluator를 원 current223개에 각각1회 호출한다. 이 인스턴스의 current policy만 계측 객체로 감싸 기존 baseline 출력도 수집하므로 baseline을 별도 OFF arm에서 다시 계산하지 않는다. 수집한 baseline 및 최종 candidate를 보존 r396 JSON의 전체 값과 각각 exact 비교한다. 필드 삭제나 tolerance가 없으며 첫 차이의 경로/값을 기록하고 중단한다. 비용 값은 과거 비교기의 바깥 계측 행이므로 명령 출력의 exact 비교 대상과 분리한다. r396의 기존 양수119와 비대상129도 그대로 대조한다. original795/legacy795/기존29검사/계획24개/새 주행은 반복하지 않는다.

반례는 같은 마지막222 입력과 원 r386 생성기의3개 변형만 쓴다. 경로 시각 만료·전방/측면 복수 점유는 원 baseline/candidate 전체 결과 및0속도를 대조한다. 시각 미검증은 기존 scene 생성 이전 실패와 같은 입력 거절로 유지하되 새 ON evaluator가 더 앞의 명시 시간 관문에서 거절한다. 이 경우 baseline/core/후보 계산0회와 `candidate_evaluated=false`를 확인하고, 기존 `MotionUnavailable/scene`과 새 `ValueError/runtime_input_timing_gate`를 별도로 기록한다. 이를 실제 STOP 명령이나 원 pipeline 성공으로 바꾸지 않는다.

고정 입력은 r386 원 archive에서 추출했으며 상세 추출 근거는 `input_provenance.json`이다. 원 current/시각/표본/마스크는 그대로이고 기존 final geometry 비교와 같은3치수만 header에 명시했다. r377 marker `.14`의5구간을 유지하며 r399 새 marker `.09`/4구간과 섞지 않는다. 원시 입력·상세 JSONL과 source archive는 기존 Git 제외 정책을 유지한다.

## 지정 실행자 명령

아래는 빌드·단위검사·저장 비교의1회 명령이다. 아직 실행하지 않는다. root가 WSL 자원을 배정한 담당자만 새 root와 receipt로 실행한다. 동일 경로가 이미 있으면 거절하며 자동 삭제/재시도하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/current_corner_runtime_r405_v1/execute_isolated.py \
  --root /home/jinhyeong/it-arena-r405-corner-20261113-a \
  --receipt artifacts/validation/2026-11-13/current_corner_runtime_r405_v1/verify_v1
```

프로젝트 소스는 동결172파일만 새 root에서 사용한다. 위 ROS setup으로 받은 `PYTHONPATH`는 독립 프로젝트 경로 뒤에 보존한다. `G1G2Node`의 정적 import에 필요한 rclpy/cv_bridge/메시지 라이브러리용이며 ROS 노드를 시작하지 않는다. 실제 프로젝트 import가 새 root 밖에서 발생하면 비교를 실패시킨다. native build120초, 단위검사120초, 저장 비교300초 상한이다. 실패 시 다음 단계로 진행하지 않으며 프로세스 그룹을 TERM5초→필요시 KILL5초로 회수한다. 준비 manifest와 추출 소스의 전후 해시, 실제 import 경로와 native binary 해시, PID/종료코드 및 사용한 경로 순서를 기록한다. 실행 중 어떤 준비 파일도 수정하지 않는다.

회수물은 `verify_v1/execution.json`, `results.zip` 안 `tests_report.json`, `results/report.json`, 로그 및 상세 rows다. 통과 조건은 source 변경0, 단위검사 skip0, baseline/candidate 각각223 exact, baseline 호출223회, 비대상129불변,3반례 조건 유지다. 원 기준 양수119→213은 기대 결과이며 이번 묶음의 확인 결과로 아직 쓰지 않는다. 실제 runtime 지연·명령 만료·코너 탈출·접촉·전체랩·실물 검증은 아니다. 주행 등록/한정1회 주행 여부는 결과 검토 후 root가 결정한다.

Bundle SHA256: `4ba5d0ca27dacee5df6f0777b1246b9f1c87010eeb5e177a05a630b25563c949`.
Bundle manifest SHA256: `37b38a097c8844952cee828e924520c33e11f11c17a605a72e97f771258ae2a1`.
전체 준비물 해시는 `manifest.json`에 기록한다. 인계 후 root의 명시적 반환 승인 전에는 사소한 문서/보고 필드도 수정하지 않는다.
