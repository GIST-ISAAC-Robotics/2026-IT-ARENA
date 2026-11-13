# r496 단일 교통 대표 실행 준비 결과

준비·회귀·동결을 완료했습니다. 이 문서 작성 시 실제 주행은 0회이며, root의 동결 원문 검토와 실행 영수증 이후에만 기존 단일 실행을 시작합니다.

- 실제 공통 경로를 import한 native 관련 6파일: **166 passed in 5.62s**. `moving_pass.py`와 `bypass_entry_trial.py`의 r494 stage/common 내용은 줄바꿈 정규화 후 동일하며, 검사 전후 생산 소스 불변과 `native_v1` 활성 상태를 확인했습니다. 기존 build를 재사용했고 추가 빌드는 하지 않았습니다.
- preflight의 공통 import·현재 구성·시설/actor 기하 확인 통과. 동결 입력 **1,362개**의 SHA와 소스 ZIP 일치를 재확인했습니다. 준비 단계에서 ROS 초기화·센서 접근·Gazebo 실행은 없었습니다.
- 첫 native 시도는 artifact runner의 `str`/`Path` 인자 오류로 **pytest 시작 전** 종료했습니다. runner만 고쳐 같은 승인 묶음을 실행했으며, `native_v1/pytest_attempt1.log`와 종료 영수증을 보존했습니다.
- 확인한 검증 PID 758·771은 모두 종료했고 잔존 소유 PID는 0입니다. WSL은 다음 실행 검토를 기다리는 유휴 상태입니다.

| 동결 항목 | SHA-256 |
|---|---|
| `prepared.json` | `e9384abd24365010bf477c32a9e179674eace018b0bfe4cf2eb74075a59be778` |
| `sources.zip` | `e60b7937ff460c3035055bfdc86f6569e117dabf9064571afbc917b5cfee60a6` |
| `native_v1/receipt.json` | `29dcd13cd4156e378d34803a718ef6e9f3e06a5511a1aadad7479871b97f96ea` |

시험 범위는 r490과 같은 `straight_moving_lead`의 GREEN 이후 10 sim초입니다. 기존 속도·시설·actor·센서·차량 기하·판정·STOP/종료 절차를 유지합니다. r489 독립 guard, r494 현재 경로 비교, r491 float 직렬화와 root 승인 이관 내용을 포함합니다. 이 준비 통과를 교통 대표 통과나 실차 검증으로 해석하지 않습니다. 원 r490 실패는 그대로 보존합니다.

근거: [준비 확인 영수증](preflight/prepared_review.json), [실제 native 실행 영수증](native_v1/receipt.json), [pytest 원로그](native_v1/pytest.log), [preflight 결과](preflight/result.json), [동결 최종 확인](preflight/final_check.json), [root 소스 선택](source_selection.json). `prepared`의 비제어 변경 `role`은 재사용 도구의 포괄 표기를 유지하며, 파일별 정확한 구분은 `source_selection.json`의 `noncontrol_roles`를 따릅니다. 이 결과 문서는 동결 이후 추가한 보고서이며 동결 입력은 수정하지 않았습니다.
