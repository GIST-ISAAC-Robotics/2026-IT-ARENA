# r456 — 실제 GUARD 정체 진입과 관측 불확실성 분리

**r452의 원 plan 381건을 실제 evaluator header·고유 제출 입력·generation/reset으로 순서대로 재생해 381/381 exact를 확인했다. 기록 누락이나 baseline 복원 오류가 이번 미진입의 원인이 아니다.** 실제 정체의 비음수 106입력에 기존 `.1` 전체 GUARD 검사를 직접 적용해도 모두 `guard_curve_rejected_toward_left`다. 입구만 넓히는 구현은 불필요 비용만 늘리므로 하지 않았다. 공통 변경·새 주행은 0이며 r452 원 실패/미호출 결과는 그대로다.

## 실제 호출에서 확인한 입구

원 trace에는 `runtime_plan_submitted/completed` 각 381건, 고유 키 381개, 양방향 누락 0, footer 오류 0이 있다. 이번 재생은 header를 별도 정책으로 재조합하지 않았으며 generation 360 첫 제출의 실제 reset_reason을 적용했다. 반복 current의 call 386건을 plan으로 잘못 중복 실행하지 않았다.

정체 이후 108회의 실제 `guard_crawl.assess` 인자를 관찰했다. baseline은 전부 양수 `.85`의 `local_follow/FOLLOW`, 현재 경로이며 selected만 `parallel_hold_reference_not_straight`로 0이었다. 그러므로 r451의 기록 normal→baseline 복원에 의존한 오판은 이 항목의 원인이 아니다. 측정속도 0이 106건, 음수 2건이며 음수는 기존 상위 절댓값 계획과 구분해 새 분기에서 거절됐다.

108건 모두 기존 `.25`는 `large_motion_masked_toward_left`, `.15`는 `toward_left`였다. `.15`의 마스크 꼬리는 이미 1 cm 안이므로, `guard_crawl.py`의 “이전 시도 모두 masked 이유·꼬리>1 cm” 입구를 만족하지 않는다. 실제 함수 AST의 각 거절 식을 실제 호출 locals로 평가해 이를 기록했다. 그런데 이 입구를 생략한 독립 `.1` 검사 106건도 양수 0이다. 이 결과 때문에 입구 확대 stage/patch는 만들지 않았다.

## 충돌 예측과 추가 관측 요구의 차이

대표는 reference sim 35.22초, generation 360/sequence 573, source 35.10200000000023초, epoch 0이다. 실제 owner/scene/baseline/selected/raw scan을 복제했으며 측방 상태나 앵커를 합성하지 않았다. 검사 전후 복제 owner 불변을 확인했다.

| 검사 | 기존 `.15` | 독립 `.1` |
|---|---:|---:|
| 반응·제동 이동 거리 | .053325 m | .034300 m |
| 전체 영역 queries | 957 | 532 |
| 일반 unobserved / 전면 mask | 0 / 0 | 0 / 0 |
| 좌측 mask 표본 | 542 | 304 |
| 현재 차체 바깥 mask 최대 거리 | 9.186 mm | 6.659 mm |
| `region_blocked` | 없음 | 없음 |
| GUARD 앵커 대비 횡방향 증가 | 13.167 mm | 11.673 mm |
| 현재 차체 대비 추가 접근 | 3.478 mm | 1.985 mm |
| `current_approach` 요구 / free | 677 / 135 | 375 / 71 |
| 부족 표본: mask / 일반 미관측 | 542 / 0 | 304 / 0 |

전체 106개의 `.1`에서도 앵커 대비 증가는 11.265–12.443 mm, 현재 대비 추가 접근은 1.976–2.427 mm, mask 꼬리는 6.562–7.161 mm다. 기존 1 cm 값은 현재 코드의 시험 파라미터이며 규정이나 실측 안전 보증이 아니다.

`pass_release_trial.py::_guard_curve_at_cap`은 경로 좌표의 차체 모서리 최대 횡위치를 GUARD 진입 앵커와 비교한다. 앵커+1 cm를 넘고 현재보다도 증가하면 `rejected_sides`를 만든다. 이후 전체 영역이 통과해도 `observed_guard_approach`가 해당 방향으로 늘어나는 모든 표본에 `context['free']`를 요구한다. `context['masked']`를 별도 제한 위험으로 수용한 앞 단계의 판단은 여기에서 사용하지 않는다. **이번 최종 거절은 유한 장애물과의 충돌을 발견한 결과가 아니라, 작은 장착 가림 표본도 전부 현재 자유 관측이어야 한다는 별도 요구다.** 가림 속 물체의 미래 접근 위험이 사라졌다는 뜻은 아니다.

대표 `.1`의 실패 304점은 현재 차체 기준 `x=.014870..032659 m`, `y=.075642..081659 m`이다. 현재 차체 외곽은 `x=±.10`, `y=±.075 m`이며, 이 점들은 차체 앞/뒤로 길게 뻗는 영역이 아니라 **좌측 옆면 바로 바깥 0.642–6.659 mm 띠**에 있다. 304점 전부 장착 mask, 일반 미관측 0이다. 전체 좌표와 분류는 [approach_geometry.json](pinpoint_v1/approach_geometry.json)에 있다. 기존 `.15`의 실패 542점도 같은 분류이며 범위는 `x=.013482..032629`, `y=.075664..084186 m`다.

## 단독인지, 실제 옆차가 있는지

r452는 두 moving actor가 있는 `two_actor_course_lap`이며 단독 시험이 아니다. 다만 대표 정체 시각의 평가 정답상 두 actor 중심은 자차에서 각각 3.105/3.667 m 떨어져 있다. 가까운 옆차로 정체한 장면으로 해석하지 않는다.

현재 왼쪽 `present`를 만든 compact 반환 4점은 로컬 `x=.0535..0684`, `y=.3932..3958 m`에 있다. 저장된 자차 pose로 평가용 변환을 하면 월드 `x=6.5161..6.5307`, `y=5.6315..5.6351 m`이다. 이는 `eval_r406_trial_old610_marker20_support`(pose `6.5275, 6.0`)의 STL 기둥 단면 경계와 **0.013–0.386 mm** 차이로 대응한다. 높이 .10/.1346/.20 m 단면 모두 같은 결과다. 따라서 이 대표의 현재 compact 반환은 이동 actor보다 **old610 marker20 지지대 기둥 반환으로 설명되는 근거가 강하다**. 초기 벽만 대조한 가설은 가까운 벽이 .181–.188 m 떨어져 있어 기각했다.

이 귀속은 평가용 정답·모델을 사용한 사후 추론이다. 센서만으로 물체 정체를 증명하거나 과거 `seen` 이력을 전부 기둥으로 다시 분류한 것은 아니다. planar pose의 roll/pitch/z가 없어 정확한 3D ray/mesh 교차 증명도 아니다. 제어기·후보에는 지도/actor 정답을 입력하지 않았다. [평가 귀속](pinpoint_v1/evaluation_attribution.json)에 원 pose·STL/월드 SHA와 좌표를 남겼다.

## 최소 후속 판단

주 담당의 구조 설계에 넘길 쟁점은 `.1` 특례 추가가 아니라 **동일 이동 영역의 제한된 자기 가림을 앞 단계에서 허용한 뒤, 누적 경로 횡추세 조건이 뒤에서 다시 전수 free로 거절하는 중복 판단**이다. 현재 입력에서는 이미 `.15`도 full region을 통과하므로 속도를 더 낮추는 것만으로 이 불확실성 관문을 없앨 수 없다.

간단한 대안의 검토 방향은 유한 반환·예측 점유와 겹치는 명확한 충돌 위험을 유지하면서, 실제 추가 이동의 mask/일반 미관측을 한 공통 기준으로 판단하고 누적 앵커·compact 기둥 반환만으로 영구 GUARD에 머물지 않도록 역할을 정리하는 것이다. 같은 판정선을 임의로 낮춰 성공 수를 만드는 수정이나 “기둥이므로 모든 측방 상태 해제”는 이번에 하지 않았다. 구체적 허용 위험/단독 우선 설계와 변경 범위는 주 담당이 결정한다.

## 검증·범위·회수

- 원 native 전체 plan 381/381 exact, 실제 정체 인자 108건, 독립 `.1` 106건 모두 원 toward 거절. 대표 `.15/.1`의 분류 계측 2회는 원 판정 함수를 그대로 호출했고 상태를 바꾸지 않았다. 새 주행·새 ROS·Jetson 사용 0.
- 재생은 r452 archive의 229파일 독립 `base/`만 사용했다. [source_manifest.json](source_manifest.json)의 SHA, 원 archive/trace 불변을 확인했다. 공통 파일/문서 변경과 정책 patch는 없다. 구현 이관 대상도 없다.
- WSL PID 749 재생은 exit 0으로 wait/회수했다. 최종 소유 프로세스 0과 입력 변경 0을 [resource_return.json](resource_return.json)에 기록해 주 담당에게 반환했다. 이후 작업은 Windows 자료 정리뿐이다.
- 첫 회수 검사의 Windows 경로 구분자 실패와, 숫자 PID가 잠시 무관한 `(udev-worker)`에 재사용되어 오탐한 원 기록을 보존했다. 파일 경로 정규화·argv 소유 확인만 수정했고 재생/주행은 반복하지 않았다. 무관한 프로세스는 건드리지 않았다.
- 벽만 조사한 첫 귀속 draft는 [evaluation_attribution_wall_only_v1.json](pinpoint_v1/evaluation_attribution_wall_only_v1.json)에 보존하며, 그 draft의 wall-face 문장은 수치로 지지되지 않아 위 최종 지지대 귀속으로 정정한다.

핵심 산출물: [재생 요약](pinpoint_v1/summary.json), [381 identity](pinpoint_v1/replay_identity.json), [실제 입구 조건](pinpoint_v1/actual_entry_checks.json), [분류·실패 좌표](pinpoint_v1/approach_geometry.json), [대표 입력 수집 코드](pinpoint.py), [분류 계측 코드](approach_evidence.py). `local_actual_snapshots.pkl`은 이 실행에서 직접 만든 로컬 복제만 보관한 것이며 외부 입력을 적재하지 않았다.
