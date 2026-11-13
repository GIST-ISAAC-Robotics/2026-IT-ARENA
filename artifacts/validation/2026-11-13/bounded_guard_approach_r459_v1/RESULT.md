# r459 — GUARD 자기 가림 분류 일치, 독립 stage 최종 결과

**원 r452 plan 381건의 exact 재생 근거를 유지하고, 수정 stage의 native381건에서 원 정체108건 모두가 기존 `.15 m/s` 단계로 수용됐다.** 그 앞273건은269건 exact·270건 명령 동일이며, 정체 직전3건만 `0→.35`로 바뀌었다. 경로·측방 상태 변화0, 기존 양수→0 변화0이다. 저장 입력의 계산 결과이며 실제 탈출·지속 가속·완주를 입증한 것은 아니다. 공통 미이관 상태로 주 담당 검토에 넘기며 원 r452 실패를 보존한다.

## 변경과 유지 범위

이관 대상은 [changes.json](changes.json)의 **6파일**이다: `pass_release_trial.py`, `current_path_recheck`와 `race_runtime_policy/retained_recheck` 각각의 `_present.py`/`_guard.py`, 신규 `test_bounded_guard_approach.py`. 원 SHA·최종 stage SHA와 절대 사본 경로를 모두 기록했다. [review.patch](review.patch)는 최소 내용 diff, [apply.patch](apply.patch)는 실제 개행을 포함한 diff다. [stage_manifest.json](stage_manifest.json)은 검증에 사용한230파일을 고정한다.

공용 helper는 옵션이 없으면 기존 strict 함수와 출력 형식을 유지한다. 실제 연결은 `selected_r181 + traffic_recovery_trial`에만 적용한다. `original_v1`의 기존 동작·absent 조기 거절, 원시 시각/epoch/부호 있는 속도, baseline 점유·간격, 운영 STOP, 독립 guard 경계는 유지한다. 새 속도 단계·앵커 상승·측방 상태 해제·고정 물체 분류는 넣지 않았다.

현재 region/config와 차체 길이·폭을 명시적으로 전달한다. 전체 region 통과·일반unknown0·전면mask0·기존1cm 예산·표본 수/분류 일치를 확인한 뒤 실제 실패 mask 좌표도 대조한다. 허용 표본은 현재 차체 앞뒤 범위 안의 옆면 바깥1cm 이내이며, side별 x-span은 기존 `known_structure.ACCEPTED_HIDDEN_WIDTH_M=.05` 이하로 제한한다. 이 .05m는 다른 기하 정의의 허용 가림 폭을 보수적 상한으로 재사용한 값이며 물체 부재나 연속 관측의 실측 보증이 아니다.

각 접근 side에 실제 free가1점 이상 있어야 하며, 부족한 표본 전부가 위 범위의 mask일 때만 통과한다. free1점은 전수 가림이 아니라는 조건일 뿐이다. 현재 관측·known_masked·bounded_mask·unaccepted_masked·일반unknown을 구분하고 `basis=observed` 또는 `observed_with_mask_budget`를 기록한다. 알려진 mask가 예산 밖으로 거절돼도 일반unknown으로 세지 않는다. mask 앞뒤의 free 개수는 진단만 남기며 양쪽 free를 새 필수 조건으로 만들지 않았다.

absent current에서는 현재 side present 등 기존 선행 검사를 유지하고 앵커 거절만 모은 뒤 원 region/접근 검사를 수행한다. rejected가 있는 선택 분기의 실패 외부 reason은 기존 `guard_rebase_toward_unreleased_side`를 보존하여 기존 낮은 목표 재검사 연결을 끊지 않는다. rejected 없는 경로의 원 반환 형식도 그대로다.

## 저장 native 비교

원본은 r456의 실제 runtime header·고유 제출/generation/reset 재생381/381 exact를 사용했다. 원 trace SHA는 `0e4718095427a7dced6c04a367ebc12f5be250ebbc619c37c0b375a77c367ee8`이며, 이번 stage도 같은381입력을 같은 순서로 처리했다. header·raw scan/속도/history를 바꾸지 않았고, 조회용 actor/지도 정답은 입력하지 않았다.

| 구간 | 입력 수 | 원→stage 결과 |
|---|---:|---|
| 원 정체, reference ≥34.999s |108|108개 모두0→.15; 경로/측방 상태 불변 |
| 그 앞 구간 |273|269개 전체 exact;1개는 관측 개수 진단만 추가;3개0→.35 |
| 전체 |381|명령 동일270, 양수259→370, 양수→0 없음 |

추가3건은 reference34.72/34.82/34.92초이며 같은 bounded mask 근거로 기존 `.35`가 수용됐다. 측정속도는 각각 `.673475/.367350/.122450 m/s`다. 예측속도는 기존 `max(계획속도, 목표)`를 유지하여 감속 완료를 가정하지 않았다. `.1` 특례를 추가하거나 정체108개 시각을 코드에 외운 것이 아니다. 바뀐111명령 전부에서 실제 mask 예산·side별free·원 측정 운동 보존을 확인했다. [112개 변경 검토](native_v1/change_review.json)에는 이111건과 진단만 바뀐1건을 전부 분리했다.

실제로 기록된 후속 current 입력2쌍(reference11.522/11.622초)은 새 stage에서도 원 `.35` 명령·이유를 유지했다. **새 양수 허용111건은 원 시험에서 plan0이어서 실제 후속 current 입력이 없다.** 해당 값을 만들거나 실제 처리한 것으로 주장하지 않았다. 현재 경로 연결의 bounded 수용·거절은 아래 두 current 계열 검사와 저장 ray 반례로 확인했으며, 새 허용 구간의 실제 current/발행/구동은 후속 주행에서 확인해야 한다.

## 검사·원 실패·남은 위험

Windows 관련143검사가 통과했고(기존125+당시신규18), 최종 신규24검사도 통과했다. 총149개 고유 검사이며 반복 실행 횟수는 더하지 않는다. 신규 검사에는 양쪽 current의 absent 순서·reason 유지, side별free, mask만 있는 side, query 앞뒤/1cm/span 초과, 일반unknown, 실제 저장 ray의 근접 전방·측방 유한 장애물/전방 unknown6경우가 포함된다. 명시 selected_r181에서도 이 실제 ray 반례는 모두0이다. 기존 original_v1 기대값은 수정하지 않았다.

처음143묶음 이전의 v1/v2 실패 로그는 보존했다. traffic_recovery만으로 proof를 넘겼을 때 original_v1 fixture2건의 `.35`가 잘못 허용되어 selected_r181 조건을 추가했다. 신규 fixture padding 누락2건도 수정했다. 주 담당의 정적 검토에 따라 absent의 무관한 반환 필드 확대를 되돌리고 known mask와 일반unknown 진단을 분리한 뒤 신규24를 다시 통과했다. 준비 단계의 Windows cp949 읽기 실패도 `preparation_v1_failure.txt`에 남겼다.

**허용 가림 쐐기 안의 얇거나 뾰족한 물체를 놓칠 위험은 남는다.** `.05` span과 side별free는 “옆차는 반드시 검출된다”는 보장이 아니다. 이 변경은 명확한 유한 장애물·일반unknown 거절을 유지하면서 작은 자기 가림을 전수free로 재거절하던 불일치를 줄인다. 실물 안전이나 전체 조건의 충돌 회피를 주장하지 않는다.

## 회수와 이관

native PID741은 wrapper에서 wait 후 exit0으로 종료했다. 사후 `/proc` 조회가 `Wsl/Service/E_UNEXPECTED`로 실패해 같은 자기범위 조회를1회 재시도했으나 같은 오류였다. 원 오류를 보존하고 재생을 반복하지 않았다. 주 담당이 기존 허용 범위에서 Ubuntu-24.04만 복구한 뒤 `check_resources.py`를 실행하여 **소유 PID0·stage/원 입력 변경0·native exit0**을 확인했다. [최종 회수](resource_return.json) 시각은2026-11-13 13:54:20 UTC다. WSL은 주 담당을 거쳐 r457 담당에게 반환했으며 이후 WSL을 사용하지 않았다. ROS/Gazebo/Jetson 신규 실행은0이다.

근거: [native 요약](native_v1/summary.json), [전체381출력](native_v1/plans.json), [실제 current2쌍](native_v1/actual_current_pairs.json), [변경 범위 검사](native_v1/change_review_summary.json), [Windows 최종24](pytest_v6/receipt.json), [관련143](pytest_v3/receipt.json), [실제정체 Windows 국소106](local_windows_v1/summary.json). `change_review_summary.json`의 사후회수 pending 표시는 서비스 복구 전 시점 기록이며 최종 상태는 `resource_return.json`이다. 공통 통합과 후속 주행은 주 담당 소유다.
