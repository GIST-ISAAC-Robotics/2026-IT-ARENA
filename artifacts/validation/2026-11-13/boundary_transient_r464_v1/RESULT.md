# r464 결과 — 동일한 현재 코너 경계 갱신

10줄 후보가 원 r457 저장 입력에서 최초 두 plan 정지를 제거했다. **원 동결 baseline 405/405 bit-exact, 후보 402/405 bit-exact**다. 기존 양수→0은 없고, 측방 seen/present/passed/vanish·selector mode·handoff 상태 변화도 없다. 이 범위에서는 좁은 수정의 통합 검토를 권한다. 실제 차량 명령 발행, 다음 센서 입력, 차체 진행 및 완주 개선은 검증하지 않았다.

## 641–645와 지속 교착 분리

| plan | 원 → 후보 속도(m/s) | 원 → 후보 조향(rad) | 결과 |
|---|---|---|---|
| 641 | 0.80 → 0.80 | −0.355733 → −0.369785 | 같은 왼쪽 벽의 현재 관측으로 갱신. source 37.703 → 37.803 s. 기존 경로 기억 대신 현재 경로를 선택 |
| 643 | 0 → 0.75 | 0 → −0.406477 | `follow_boundary_unconfirmed` → `local_follow`. 641 갱신으로 기존 TTL 안에서 현재 경계 유지 |
| 644 | 0 → 0.45 | −0.30 → −0.30 | `follow_release_wait` → `local_follow`. 앞선 경계 HOLD가 생기지 않아 추가 대기도 발생하지 않음 |
| 645 | 0.75 → 0.75 | −0.457844 → −0.457844 | 전체 출력 exact |

641 이전 302행과 645 이후 99행은 모두 전체 출력 exact다. 경로 변화는 641의 기억 경로→현재 관측 경로와 643의 경로 없음→현재 왼쪽 경로뿐이다. 선택 벽 메타데이터 차이는 643의 없음→left 한 건이며 **left↔right 전환은 없다**. 644는 경로 자체가 원본과 같고 HOLD만 발생하지 않는다.

계획의 속도·각도 변경은 원 입력을 유지한 저장 재생 결과다. 특히 644의 previous steering 0 및 측정속도는 실제 원 주행 값이며 후보의 가상 물리 응답으로 바꾸지 않았다. 따라서 후보 641/643에서 다른 출력이 실제 발행됐다면 뒤의 관측도 동일했을 것이라고 가정하지 않는다. 원 643/644에는 current 재검사 입력이 없으므로 새 입력을 합성하지 않았다. 이후 100건 자기마스크 정지는 별도 r463 범위이며 여기서 current를 다시 계산하거나 수정하지 않았다.

## 실행과 불변 확인

- WSL Ubuntu-24.04, Python 3.12, 원 archive에 포함된 native `.so` SHA `99bb9a7def33470a6cabfe5195660cc178f16f60c72d4133d389a10ee285d434`를 복사해 사용했다. C++ 소스와 binary 모두 원 archive에서 가져왔고 새 빌드·공통 설치는 없다.
- baseline/후보 각 228파일의 유일한 차이는 `local_follow.py`다. 각각 별도 프로세스에서 실제 `runtime_evaluator_header`, 고유 제출 405개, generation361 첫 제출 sequence1의 원 `reset_reason=null`을 그대로 사용했다. 내부 상태를 원 출력으로 채우지 않았다.
- baseline은 허용 비교뿐 아니라 **Python 출력 구조 전체의 정확 일치 405/405**였다. 후보는 402/405 정확 일치다. 기존 양수→0=0, 측방 상태 변화=0, selector mode 변화=0, handoff 상태 변화=0.
- epoch는 405행 모두 원 0, signed 정책 `forward_drive`, acquisition 정책 `verified_only`를 유지했다. 두 재생의 입력 순서 SHA는 동일한 `9ea6820d5f7613ab8fd18d67d691a86c79fca3a8dc41387155f04b6291fe7296`이고 호출 전후 입력 변경0이다.
- 각 실행 후 패키지 hash 변경0, 격리 경로 밖 프로젝트 import0, trace SHA 불변을 확인했다. 공통 소스/문서 변경은 없다.

국소 회귀는 9개를 확보했다. 같은 벽 갱신, 프로필 OFF/만료/다른 벽/이탈/같은 scan/epoch의 원 결과 보존, 실제 전방 점유 정지, 기존 HOLD의 재개 확인을 포함한다. **첫 두 시도는 8통과/1실패**였다. 실패는 합성 장애물 fixture가 추가한 광선에 source angle을 추가하지 않은 오류와 취득 시각을 회전 끝에 붙일 때의 순서를 맞추지 않은 오류였다. 모두 `LocalScene` 생성 단계에서 제어 호출 전에 거절됐다. 각 원 test와 실패 로그를 보존했고 fixture만 정정했다. 정책 파일은 바꾸지 않았다. 수정 후 해당 1개만 다시 실행해 통과했고 앞서 통과한 8개를 불필요하게 반복하지 않았다.

## 비용·회수·한계

원본/후보를 한 번씩 순차 실행한 WSL 진단 비용은 다음과 같다. 반복 계측이나 엄밀한 성능 비교가 아니며, 전체 지연을 단일 추가 호길이 비교 비용으로 귀속하지 않는다.

| WSL plan 계산 | 중앙값(ms) | p95(ms) | 최댓값(ms) |
|---|---:|---:|---:|
| baseline | 16.737 | 27.451 | 99.299 |
| 후보 | 17.897 | 30.174 | 111.061 |

Windows 기하 진단 비용 및 Jetson 비용과 섞지 않았다. 추가 코드의 실행은 현재 같은 벽의 기존 x 비교 실패 시 호길이 비교 한 번이며, 시각·TTL·점유·제동·독립 guard 조건은 그대로다. 같은 벽 라벨은 객체 동일성을 증명하지 않고 기존 호길이 비교의 가까운 공통구간 한계도 남는다.

첫 두 pytest 묶음은 각각 exit1과 자식 PID 회수로 보존했다. 세 번째 WSL 재진입은 3분 넘게 무응답이고 실행 영수증이 없었다. 소유 wsl.exe 두 개의 ID/시작 시각/명령을 확인한 후 해당 호출만 종료했다. 이 시점의 Linux PID 감사 미완료는 `wsl_entry_hang_v3.json`에 그대로 남겼다. root의 기존 승인에 따라 **Ubuntu-24.04만 terminate exit0 → 재진입1회**로 복구했고, 이후 최종 묶음이 실제 시작됐다. 다른 distro/WSL 전체 서비스/앱은 종료하지 않았다.

최종 수정 반례 PID706, baseline PID760, 후보 PID766은 모두 exit0·`/proc` 소멸 확인을 마쳤다. 마지막 `/proc` 감사에서 r464 잔류·경쟁 실행은0이고, 그 감사를 실행한 운반 Python/bash도 외부 명령 exit0으로 종료됐다. **WSL 소유권은 root에 반환했다.** 종료 후 Windows 감사에 별도 `codex --version` WSL 프로브가 나타났지만 r464 소유가 아니고 최종 묶음 종료 뒤 시작되어 변경하지 않았다.

## 근거

- [변경 diff](changes.patch), [원본/stage SHA](changes.json)
- [최종 묶음 영수증](execution_v1/bundle_receipt_v3.json), [원본 요약](execution_v1/baseline_result/summary.json), [후보 요약](execution_v1/candidate_result/summary.json)
- [405행 전체 차이 집계](execution_v1/comparison_summary.json), [초기 4개 plan](execution_v1/candidate_result/focus_641_645.json)
- [첫 fixture 실패](execution_v1/pytest9.log), [두 번째 fixture 실패](execution_v1/pytest9_v2.log), [수정 반례 통과](execution_v1/pytest_remaining1_v3.log)
- [WSL 진입 무응답](execution_v1/wsl_entry_hang_v3.json), [승인된 배포판 복구](execution_v1/wsl_recovery_v3.json)

공통 통합과 다음 주행 판단은 주 담당이 맡는다. 이번 결과를 기존 r457의 미완주/원 실패 판정에 소급 적용하지 않는다.
