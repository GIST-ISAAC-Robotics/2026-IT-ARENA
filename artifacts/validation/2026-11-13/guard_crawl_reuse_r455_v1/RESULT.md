# r455 현재 GUARD 호출의 view 재사용 — 봉인한 독립 stage

2026-11-13, GPT. **같은 `_select`에서 이미 만든 view를 추가0.1 검사에 전달하는 소규모 대안을 구현했다.** 공통 소스는 r452 실행 중 동결 상태로 유지했다. 이관 대상은 소스2파일+신규 회귀1파일이다. Windows 고유168검사와 r451104행 비교를 마쳤다. 최신 사용자 우선순위에 따라 여기서 결과를 봉인하며 최적화/계측을 확대하지 않는다.

## 변경과 판단

r454에서 새 adapter 전체는18.556ms, 실제0.1 GUARD는11.210ms였다. 나머지7.361ms에는 중복 `threat_view` 등 연결 비용이 포함됐지만 각각의 함수 비용을 분리한 결과는 아니었다. 이번 변경은 **명확히 중복되는 view 구성1회만 제거**한다.7.361ms 전부를 절약했다고 주장하지 않는다.

- `guard_crawl.assess`에 내부 keyword `current_view`를 추가했다. 전달되면 사용하고 직접 helper 호출에서 생략하면 기존 계산을 유지한다. 미전달과 계산된None을 구별한다. eligibility·signed 측정·전체 GUARD/미관측/1cm 조건과 결과 메타데이터는 그대로다.
- `released_plan.enabled`가 기존 `_select`/command와 함께 `_guard_curve`를 일시 감싼다. 실제 `_guard_curve` 인자의 view를 **현재 `_select` 호출에만 속한 ContextVar frame**에 전달한다. owner/scene/baseline/raw_scan은 같은 객체, speed/previous/dt는 같은 값일 때만 기록한다.
- frame은 원 `_select` 반환/예외 직후 reset한다. 후보 평가는 기존처럼 원 `_select` 뒤이므로 observer의 원선택/후보와 shadow의 원반환 의미를 유지한다. owner/scene 캐시, 스캔 간 저장·TTL·새 설정은 없다.
- 종료 시 `_select`, `_guard_curve`, `BypassHandoffFollow.command`를 진입 시 메서드로 복구한다. 기존 signed `_MEASURED` 범위는 불변이다. 기존 단일 Python 과정의 정책 연결이며 새 다중 스레드 보장은 추가하지 않는다.

현재 호출의 중복을 제거하고 출력을 보존한다는 근거는 확보했다. 주 담당은 diff/168검사/104행 일치를 검토했으나 **실패가 아닌 채택 대기**로 결정했다. r452에서 이 후보 호출이0이고 정책 재설계로 helper 자체가 바뀔 수 있어 현재 hotpath에 hook을 추가할 실익이 확정되지 않았다. 실제 Jetson 감소량은 미측정이며 추가 계측/공통 통합은 하지 않는다. 상위 `pass_release_trial.py` 수정은 필요하지 않았다.

## 검증과 경계

| 범위 | 결과 |
|---|---|
| 기존 관련 회귀+신규10개 |최종 고유168개 통과, 미해결 실패0 |
| 실제 `_select`→`_guard_curve` 연결 |실제 메서드와 명시 합성 기하 fixture에서 view1회, 후보 `.35/.25/.15/.1`, reason/behavior/path·측방 상태 보존 |
| shadow/observer |원선택0/후보0.1 구별, shadow는 원선택 반환 |
| RELEASED |기존 저속 분기와 통과 이력 유지, GUARD view 전달 개입 없음 |
| 중첩·예외 |중첩3조합, guard/observer/command 예외에서 frame·메서드 복구. 음수 원측정은 abs 전달과 구별해 후보 거절 |
| 호출 격리 |새 호출/다른 owner에서 이전 view 재사용 없음 |
| r451 원104행 |원 세 검사 진단1e-12·정수/구조 exact, 구/신 adapter **전체 출력 exact104/104**, 수용104/current dispatch 지원104, 상태·입력 불변 |
| 저장104행 view |기존208회→stage104회. 후보 검사는 생략하지 않음 |
| 최종 patch |LF patch의 `git apply --check` exit0 |

현재 지원104는 `_present.supported` 계약이며 독립 current 계산·부모 발행·보호층 실행 증거가 아니다. 저장104는 원 trace의 **갱신 후 국소 GUARD 상태와 scene/path**를 `_guard_curve` hook으로 전달한 비교다. 누락된 전체 plan 이전 상태는 채우지 않았다. 실제 `_select` 상태 갱신/분기 연결은 별도 합성 회귀로 확인했으며 두 범위를 전체plan 재생으로 합산하지 않는다.

Windows `numeric_kernels=python`, project import는 stage 안에서만 이뤄졌다. 저장 비교5.628초는 검사 소요시간이며 성능 개선율이 아니다. Jetson/WSL·주행·장치·설치·기본 설정 변경0이다.

## 보존한 초기 실패

첫 회귀는164통과/4실패였다. 기존 검사에 필요한 `g1_partial_trial`, `test_parallel_lane_trial`, `test_g1_partial_policy`의 격리 사본 누락이었다. 원 로그/receipt를 보존하고 보조 파일을 넣은 뒤 **실패4개만** 재실행해 통과했다. 판정 코드는 바꾸지 않았다.

초기 신규 회귀는 원본 policy 사본과 직접 비교했다. 최종 이관 테스트가 artifact에 의존하지 않도록 실제 분기/출력/observer/복구 계약을 직접 확인하는 독립 회귀로 정리하고 해당10개만 재확인했다. 원본 대 stage exact는 별도 `verify_saved.py`와104행 결과에 남긴다. 초기 시험 소스가 든 `staged_sources.zip`은 보존하며 **최종 이관 archive는 v2**다.

첫 patch는 Windows CRLF 직렬화로 적용 검사가 거절했다. `apply_crlf_v1.patch`를 보존하고 LF 내보내기만 수정했다. 최종 적용 검사는 통과했다.

## 인계

- [대상3파일 원본/stage SHA](changes.json), [최종 diff](apply.patch), [stage](stage/).
- [최종 archive v2](staged_sources_v2.zip), SHA256 `7b2faec5510f708712bec05252b682f04afbe4a0a04e83d1ff90aadaeffe717f`.
- [최종234파일 manifest](stage_manifest_v2.json), [공통 원본229파일 SHA](original_sha256.json). 공통 출처 변경0, 대상 밖 stage 원본 변경0.
- [104행 보고서](saved_v1/report.json), [원 결과](saved_v1/rows.jsonl), [최종 집계](summary.json).
- [최초 회귀](verify_v1/receipt.json), [실패4개 회수](verify_v2/receipt.json), [최종 독립 신규10검사](verify_v3/receipt.json).

Windows 명령은 모두 종료했고 새 background 프로세스0이다. Jetson/WSL 소유권을 취득하거나 사용하지 않았다. 추가 계측/공통 통합 없이 주 담당에게 봉인 결과를 인계한다.
