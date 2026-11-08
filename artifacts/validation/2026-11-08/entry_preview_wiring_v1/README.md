# 정지차 짧은 목표점 후보의 실행 연결 사본과 r36 준비

2026-11-08. **r35 실행 종료 후 검토한 사본을 r36으로 생산 소스에 적용했다. 기본 OFF이며 관련405검사·연결 검사·2패키지 빌드·실제 import 7파일 해시를 확인했다. 같은 STOP/START 실제1회를 완료했으나 재개 후 0.527164m로 1.2m 목표에 미달했다. 원 실패 판정은 유지한다.** 후보 5모듈은 [entry_preview_probe_v1](../entry_preview_probe_v1/README.md)의 overlay를 그대로 사용한다. r33의 1.2 m 미달(0.50553 m) 실패는 그대로 보존한다. r33의 898개 입력 무결성·정상 종료는 통과했으며 이를 실패로 표현하지 않는다.

## 연결 사본

- `build_wiring.py` → `staged/src/...`: `g1_g2_node.py`는 `g1_g2_adaptive_entry_preview`(기본 `False`)를 `g1_g2_stopped_entry_centering` 바로 뒤에 선언하고 `HandoffExecutionConfig(adaptive_entry_preview=...)`로 전달한다. `simulation.launch.py`는 `evaluation_adaptive_entry_preview`(기본 `"false"`)를 선언하고, `reserve == "off"`이면 거부하며, 기존 `g1_g2_stopped_entry_centering`과 같은 `controller_parameters.update` 블록에 전달한다. 원 CRLF를 유지하며 추가만 한다.
- `wiring_manifest.json`: 연결 2파일 before/after와 overlay 5모듈 before/after 해시.
- `check_wiring.py`(`check.sh`): AST로 기본 false·양끝 이름·전달 경로·거부 조건, overlay import로 실행 설정 기본 false, r33과 같은 조합+후보 ON 허용, 문자열 값 거부를 확인한다. src는 before 또는 after 상태만 허용한다. 결과는 `check_result.json`.

## r36 실행 준비

- `run.py`·`assess.py`는 r33 도구를 복사하지 않는다. `r33_tools.py`가 r33 `run.py`·`assess.py`·`settle.py`의 해시를 r33 등록값과 대조한 뒤 이 폴더를 출력 위치로 실행한다. `run.py`의 유일한 차이는 `'evaluation_stopped_entry_centering:=true'` 뒤 `'evaluation_adaptive_entry_preview:=true'` 추가다(`python r33_tools.py --dry`).
- 시나리오·정지차 43.26 m·첫 STOP 42.26 m·1.2 m/25초·0.5초 연속 정착·접촉/노면·정상 종료 판정은 r33과 같다. `assess.py`는 r33 판정을 먼저 실행하고 후보 설정 ON과 정렬 호출의 속도0·3 rad/s 변화율만 별도 게이트로 추가한다. 정렬 호출은 우회 성공으로 세지 않는다.
- **r33과의 차이는 두 가지다.** 후보 ON, 그리고 r35 `piecewise_wall.py` 잡음 분할이 부모 소스에 포함된다. r33도 `g1_piecewise_wall_trial:=true`이므로 결과 변화는 어느 쪽에서도 올 수 있다.
- `register.py`는 7파일 after 해시가 src와 일치할 때만 등록한다. `run.sh --check`는 등록·실행 없이 전제만 출력한다.

## 적용 순서(GPT 검토 후)

1. r35 실행 종료·결과 회수, `pgrep` 잔여 프로세스 0 확인. 설치본은 src 링크(egg-link/launch 심볼릭)라 r35 중 src를 바꾸면 새 프로세스에 바로 반영된다.
2. `bash run.sh --check`로 src가 `before 6/6`인지 확인. 다르면 사본을 덮지 말고 새 부모에서 `build_wiring.py`/overlay를 다시 생성한다.
3. `apply_reviewed_candidate.py --check`로 전체 before·after·부모 해시·구문·시험 종료를 먼저 확인한 뒤 같은 도구로 적용했다. 기존 모듈의 CRLF는 보존하고 신규 모듈만 LF다. 적용 전 파일은 `before.zip`에 보존했다.
4. 표적 회귀(`entry_preview_probe_v1/test_adaptive_entry_preview_staged.py`의 src 기준판, `tests/test_stopped_entry_centering.py` 등)와 OFF 재생 불변 확인, colcon build.
5. `config/tests/adaptive_entry_preview_r36_sha256.json`(parent `piecewise_noisy_corner_r35_sha256.json`)과 `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS`를 적용 도구에서 함께 등록했다.
6. `bash run.sh --check` → `after all 7/7`·`no simulation`·`run1 absent` 확인 뒤 `bash run.sh` 1회.

검증 근거: [405검사](regression_result.json), [빌드](build.log), [설치 import 7파일](installed_check.json). 최종 사본 OFF1,054판단 일치는 [재생 결과](../entry_preview_probe_v1/replay_overlay.json)이며 생산 적용 파일이 그 사본 해시와 같다.

## 실제 결과 — r36 1회

- 16:40:52–16:48:15 KST, 등록 914입력 불변·잔여 프로세스0·정상 종료. START는 약0.428초에 허가됐고 같은 경기 바인딩/RESET없음/명령 순서를 확인했다.
- 원 기준점 `before_resume` t11.349의 방향으로 투영한 최대 진행은 **0.527164m**(목표1.2m), 재개 뒤 누적 경로0.557890m다. `post_resume_progress` 성공 항목이 없다는 것을 변위0으로 해석하지 않는다.
- 후보150호출 중 양수 짧은 목표점49·정지 조향101, 정렬 속도/변화율 위반0. 양수 출력은 짧게 발생하고 후속 호출이0으로 돌아가는 양상이 남았다. 이 호출 수를 실제 우회 성공으로 세지 않는다.
- 접촉0·최소 표본 간격0.150687m·바퀴 노면 이탈/벽 표본0. 첫 정지창0.5초/245표본, 최대0.03917m/s·변위0.004081m 통과.
- 원 `dense_stop=false`는 첫 정지 실패가 아니다. 진행 조건 실패로 예정된 두 번째 `final_stop_hold`에 미도달했다. 실패 정리용 `failure_stop_hold`는0.5초/251표본·최대0.001659m/s·변위0.000156m로 통과했으나 원 기대 시나리오 완료로 대체하지 않는다.
- 관련 원본: [assessment](assessment.json), [report](run1/report.json), [실행 종료 코드](run.exit). r33의0.506m 실패도 보존. 현재 선택형은 기본OFF이며 현실 서보 도달·실차 운영의 증명은 아니다.

다음은 짧은 주시거리와 기존 주시거리의 전환, 새 스캔과 같은 스캔의 나이 증가에 따른 점유 여유/0명령 전환을 같은 저장 입력에서 분리한다. 원인 변화 없이 같은 시험을 반복하지 않는다.
