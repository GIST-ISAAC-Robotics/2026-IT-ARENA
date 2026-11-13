# r469 r467/r468 통합 후 단독1회

실행·회수 완료: 원 판정 false, 진행 9.90 m, 8 sim초 무진행 소유 STOP 1회, 자식 14개 정상 회수 및 입력 1347개 불변. WSL 반환 후 원 로그 집계까지 봉인하고 사용자 일시 중지에 따라 종료했다. [최종 결과](RESULT.md), [영수증](final_receipt.json). 아래 명령은 실행 이력이며 자동 재실행 지시가 아니다.

r465의 actor0·old610 시설3개·차량/C1/속도/물성·RTF .25·80 sim초·8 sim초 무진행 소유STOP·원 평가/종료 기준을 유지한다. 원 r465 raw 실패·run1·등록·PID·빌드/실행 영수증은 복사하지 않았다. 새 실행을 성공까지 반복하지 않는다.

원본13개 recipe는 byte exact다. config의 scope/entry와 runner/helper의 자기 경로·표시·baseline 참조만 바꿨다. 날짜 폴더 변경 때문에 `run.py`의 old610 fixture 참조를 원 `artifacts/validation/2026-11-13/old_signal_610_r326_v1`로 고정했으며 원 fixture/mesh SHA도 검사했다. 기하/시설 내용은 바꾸지 않았다. [recipe manifest](recipe_manifest.json), [검사](recipe_check.log), [차이](recipe_review.patch).

실제 공통 delta는 r465 동결 대비 `known_structure.py`, `pass_release_trial.py`, `structure_region.py`, `race_runtime_policy/current_region.py`, `tests/test_released_structure_region.py`의5개다. `prepare.py`가 이 집합과 r468 integration의 before/after를 강제 대조한다. r466/r470 stage는 포함하지 않는다. [source scope](source_scope.json), [source 검토 영수증](root_source_review.json).

주 담당은 통합184 관련검사 회수와 WSL 반환 후, 담당의 exact5 대조·빌드·동결·최종 자체 검토에 이어 단독1회 실행을 명시 배정했다. `root_source_review.json`과 후속 `root_execution_review.json`에는 이 배정과 실제 검토 수행자를 구분해 남긴다. 불일치하면 실행 전에 중단한다.

고정 준비 명령:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_full_lap_r469_v1/build_v1/prepare_once.sh'
```

`arena_autonomy`1패키지 빌드→실제 import/native SHA→고정 CLI의 ROS init 직전 차단→archive/입력 동결→검사·회수다. 기존 정책 회귀를 중복 전수 실행하지 않는다.

최종 준비·무결성 확인 뒤 고정 실행 명령:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_full_lap_r469_v1/execution_v1/execute_once.sh'
```

관측은 `execution_v1/observe.py`, 소유 STOP은 `execution_v1/stop_once.sh`1회, 회수는 `execution_v1/check_owned_processes.py`다. run argv는 소유 STOP과 동일한 `python3 <상대 run.py>`이며 bytecode는 환경변수로 끈다. 실제 진행/정체·접촉·센서/명령기한·완전 plan 제출/완료·원 report false/선행 latch·자식 종료·입력 불변을 분리한다. WSL 진입 오류는 receipt/등록/소유PID를 먼저 확인하여 중복 실행을 피한다.

최신 실행 상태는 [RESULT.md](RESULT.md)에 기록한다. 빌드나 저장 입력 회복을 완주·실차 검증으로 확대하지 않는다.
