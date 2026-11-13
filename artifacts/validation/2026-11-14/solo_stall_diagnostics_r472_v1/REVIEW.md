# r472 r469 정체 입력의 제한 진단

동결 r469 source로 **217개 plan 전체 exact, 최초 실제 current exact**를 확인했다. 공통 소스·시각·원 scan·정책 출력은 바꾸지 않았다. `diagnose.py`는 원 `propose/check/motion_region`을 호출한 결과를 별도 목록에 모으는 진단 wrapper이며, stage의 원 소스 338개도 byte 불변이다. 새 시뮬·ROS 초기화/발행·Jetson 실행은 없다.

## 확정된 하위 사유

최초 current 19.026 s의 0은 실제 속도 .367350252 m/s가 body-gap 입구 .35를 넘어 `invalid_motion_or_scan`으로 반환된 것이다. 계획 19.02 s의 실제 속도 .306125210은 범위 안이었다. raw scan 존재·dt .086 s·목표 .35이며, 이 거절은 영역 계산보다 앞선다. [실제 current 원 입력](selected_inputs.json), [exact 결과](diagnostic_v1/summary.json).

후속 19.22–29.32 s의 102개 plan은 실제 속도 0이었다. 모든 호출이 세 후보를 검사했고 수용은 0이다. 실패한 시도를 원 `propose`가 버려서 주행 로그에는 이 세부 사유가 없었지만, 동작 불변 재생으로 확인했다.

| 목표 m/s | 102개 모두의 proof.reason | 일반 unknown query 수 | masked tail 범위 |
|---|---|---:|---:|
| .35 | adjacent_structure_outside_scope | 265–374 | 13.925–15.240 mm |
| .25 | adjacent_structure_outside_scope | 200–276 | 13.653–14.776 mm |
| .15 | not_one_declared_mount_sector | 135–186 | 10.365–11.029 mm |

세 후보 모두 최종 `blocked=large_motion_region_unobserved`, 전면 mask 0이다. .35/.25는 인접 구조 공백 helper의 목표 .15 범위를 넘고, .15는 실제 masked index 집합이 선언 섹터 전체와 다르다는 이유로 거절된다. 현재는 일반 unknown 증명이 먼저 실패하므로 가림 꼬리만 합성해도 바로 통과한다고 말할 수 없다. [후보별 집계](candidate_summary.json), [원 진단 사건](diagnostic_v1/diagnostic_events.json).

## 일반 미관측의 위치와 원 반환

첫 19.22의 365/2850 unknown은 현 차체 좌측 **x=-4.86…6.78 mm, y=75.52…82.71 mm**, 즉 옆면 밖 .52…7.71 mm의 좁은 띠였다. 앞/뒤 범퍼 바깥 query는 0이다. 분류는 unknown|return 32, unknown|unknown 205, masked|unknown 128이다. 원 scan의 unknown 광선은 **71–73의 +Inf 3개뿐**이고, 인접 원 각도는 약 133.27°→130.25°이다. nearest angular bracket와 padding 분류를 구분했으며, 알려진 유한 반환의 padding 안에 든 query는 0, 최소 여유는 .3648 m다. 이것은 저장 scene에 관한 사실이며 물리적으로 자유 공간이 증명됐다는 뜻은 아니다.

.15 후보의 원 query 영역을 동일하게 복원한 첫/중간/끝 세 대표는 다음과 같다. 모두 `region_exact=true`다.

| reference | unknown / 실제 masked | 해당 선언 sector membership | 성분 양옆 반환 m | query reach / 원점 span | 전체 각폭 | padding 포함 기존 chord / query 기반 shadow 폭 |
|---|---|---|---|---|---:|---:|
| 19.22 | 71–73 / 74–77 | 74–94 | 11.020 / 7.552 | .11135 m / .205 mm | 5.805° | 22.77 / 11.48 mm |
| 24.32 | 70–72 / 73–75 | 73–93 | 11.015 / 7.064 | .11131 m / .000149 mm | 4.896° | 19.20 / 9.51 mm |
| 29.32 | 70–72 / 73–88 | 73–92 | 11.725 / 1.245 | .11025 m / .000501 mm | 14.184° | 55.25 / 27.22 mm |

세 경우 모두 **한 선언 섹터의 일부만 실제 연속 mask 성분에 포함**됐다. 여러 선언 섹터를 unknown이 잇는 경우도, 후방 cone 등 미선언 masked가 섞인 경우도 아니다. 차이 집합과 네 선언 sector의 membership은 [component_evidence.json](component_evidence.json)에 모두 있다. 해당 sector 중심은 +123.152834°이며 nominal flank 한쪽은 +Inf라 `classify(...).explained=false`다. 실제 짧은 연속 성분의 양옆은 유한 반환이라는 점과 구분해야 한다.

표의 두 폭은 서로 다른 계산이다. 기존 인접 helper의 chord는 `2*(query reach+max padding)*sin(angle/2)`, 구조 shadow 방식의 진단 폭은 `2*query reach*sin(angle/2)+origin span`이다. 후자는 실제 unknown+masked 성분 전체에 적용한 **진단 계산**이며 현행 helper가 발급한 허가가 아니다. 마지막 대표의 기존 chord는 5 cm를 넘는다. 단순 집합 equality만 바꾸면 모든 후속 조건이 해결됐다고 결론 내릴 수 없다. 근접 실제 반환·미선언 결측·과도한 숨은 폭·전면 가림은 별도 검사를 계속 요구한다.

## 엔코더 속도 창과 기존 선택값

r469의 정확한 설정은 모터 28 count/rev, 기어비 **11.911764705882353**, 바퀴 반경 .0325 m, sample 100 Hz, 임시 시뮬 지연 2 ms, wheel scale 1이다. `SimWheelEncoder._motor_state`는 직전 발행 표본과 현재 표본의 **capture timestamp 차이**로 정수 count 차분을 나눈다. `lidar_motion_ros.on_drive_motor`가 기어비와 반경을 적용하고, runtime snapshot은 마지막 수용 속도를 사용한다. 이 경로에 추가 평균 창은 없다.

plan의 마지막 표본은 **19.014 s / .306125210 m/s**, current의 마지막 표본은 **19.024 s / .367350252 m/s**이며 인접 표본은 .010 s 간격이다. 한 count 환산은 .061225042014404256 m/s다. 원 trace에는 누적 counter 위치가 없으므로 5→6 count는 속도에서 역산한 값으로 표시한다. 실제 물리 속도가 그대로였는지, 계측 양자화만 있었는지는 이 자료로 확정하지 않는다.

기존 header `follow_config.current_lane_headroom_mps`는 **0.061225042014404256**으로 이미 명시됐다. 이는 허용 입력 범위를 설계할 근거이며 실측 속도를 .35로 clamp하거나 제동/이동 계산에서 지우는 근거는 아니다. [표본과 계산](encoder_window.json).

동결 source 위치: `sim_wheel_encoder.py:113,157–168,214–223`, `drive_feedback.py:96–107`, `lidar_motion_ros.py:135–151`, `lidar_motion.py:97–100`, `runtime_node_backend.py:116–118`. 이 파일들은 `stage/src/...`의 r469 원본이다.

## 두 `_gaps.py`의 실제 연결

- 계획 RELEASED: `PassReleaseSelector` → `released_body_gap.propose` → `released_body_gap.check` → **`current_path_recheck._gaps`**. body-gap/인접 helper import가 이쪽에 고정돼 있다.
- r469 현재: 실제 Evaluator의 core module은 `race_runtime_policy.observation_policy`이고, 이 모듈의 `common`은 `race_runtime_policy.retained_recheck`다. RELEASED `_released.rebase`가 원 계획의 body-gap 수용을 보고 **공유 `released_body_gap.check`**를 다시 부르므로 이 사례도 `current_path_recheck._gaps`로 간다.
- `retained_recheck._gaps`는 그 패키지의 absent GUARD `_guard.py:120–128`에서 기존 proof policy를 다시 검사하는 경로다. 반면 `observation_policy.same_path_risk`의 plan GUARD와 현재 relative/corner 관련 helper들은 `current_path_recheck._gaps`를 import한다. 두 사본은 실제 연결점이 다르며 dead code로 취급하지 않는다.

기본 외부 `current_path_recheck` 패키지도 자체 `_guard`가 자체 `_gaps`를 사용한다. `relative_neighbor` 옵션은 그쪽 사본에만 있다. 이번 진단은 경로를 합치거나 이름을 바꾸지 않았다.

## 최소 후속 반례 준비와 실행 경계

원 실제 plan 3개와 최초 actual current 1개를 [selected_inputs.json](selected_inputs.json)에 보존했다. 추가 반례는 [counterexample_recipe.json](counterexample_recipe.json)에 변경 위치와 기대 경계를 명시했으며 **아직 실행하지 않았다**. 새 helper를 구현할 주 담당이 같은 현재 query/scan의 증명 결합, 근접 반환, 미선언 NaN, 다른 reference/epoch 근거 혼용만 우선 확인할 수 있다. 단순 예외 누적이나 예전 실패를 성공으로 바꾸는 목적이 아니다.

원217 재생은 exit 0. 대표3 보조 Python은 결과파일과 완료출력을 남겼으나 Windows `bash -c` 종료 변수 인용 오류로 wrapper가 exit 1이었다. 원 log와 빈 exit 파일을 보존했고 계산은 반복하지 않았다. 이후 별도 LF wrapper로 PID/무결성을 확인해 exit 0으로 회수했다. [자원 반환](resource_return.json): 소유 Python PID 0, stage338 불변, WSL 반환. 후속 작업은 Windows 문서/원 로그 집계뿐이다. r469 원 실패와 sources.zip은 보존한다.
