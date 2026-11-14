# r545 새 조향 상한과 코너 경로의 연결 검토

## 결론

새 `max_steering_rad=0.28475436568260193`은 최종 clamp뿐 아니라 관측 코너 경로의 최소 반경 검사와 선택형 예측에도 전달된다. 그러나 **현재 선택한 piecewise 경로는 반경을 새 상한에 맞춰 늘리거나 회전을 미리 시작하지 않는다.** `wall_offset_m=0.505`가 최소 반경을 만족하는지만 확인한 뒤 원래 0.505 m 원호를 그대로 만든다. 일반 곡선 fallback에는 곡률 실현 가능성 제약이 없다. Pure Pursuit의 목표점 선택도 조향 상한과 독립이다.

이는 구조 확인이며 r544의 실패나 원인을 판정한 것이 아니다. r475의 486개 양수 명령 중 95개가 새 상한을 초과한 자료는 저장 명령분포이지 새 상한에서의 재생·폐루프 결과가 아니다. 공통 소스/기하를 수정하지 않았으며 WSL·Jetson·Gazebo·광범위 시험을 실행하지 않았다.

## 수치와 물리 의미

자전거 모델의 후륜축 최소 반경은 `R_min = wheelbase / tan(max_steering)`이다. 휠베이스 0.128 m에서 기존 0.45784360 rad는 0.259758 m, 새 상한은 **0.437295 m**다. 최대 곡률은 **2.286788 m^-1**로 줄었다.

현재 `wall_offset_m=0.505`는 새 최소 반경보다 0.067705 m 크므로 `piecewise_wall.corner_path()`의 선행 반경 검사에 걸리지 않는다. 단순 0.505 m 원의 자전거식 중앙각은 14.223°로 새 상한보다 작다. 다만 경로는 차체 원점 좌표로 표현되고 Pure Pursuit는 후륜축에서 쫓으며 실제 이탈·방향 오차·차체 외곽이 있으므로 이 비교만으로 추종 또는 코너 통과가 보장되지는 않는다.

속도를 낮추면 횡가속도, 반응 중 이동거리, 조향 변화율 지연은 줄지만 위 기하 최소 반경은 줄지 않는다. 현재 3 rad/s 명령 변화율에서 0→새 끝각은 94.9 ms, 좌끝→우끝은 189.8 ms다. 1 m/s이면 각각 약0.095/0.190 m를 이동하는 시간이다(명령 변화율만의 계산이며 실제 서보 지연 실측이 아님). 따라서 늦게 돌기 시작한 차량을 저속만으로 원래 좁은 곡선에 복귀시킬 수 있다고 단정하면 안 된다.

## 좁게 확인한 코드 경로

| 위치 | 새 상한의 사용 | 남은 성질 |
|---|---|---|
| `local_pursuit.py:178`, `:503` | FollowConfig에 상한 전달, 엔코더 속도로 LocalFollow 호출 | 속도 입력은 명령값이 아닌 관측값 |
| `local_follow.py:824` | `minimum_radius=wheelbase/tan(max_steering)`로 `local_path` 호출 | `piecewise_wall_trial` 선택 때 적용 |
| `piecewise_wall.py:23/31/68` | `offset < minimum_radius`이면 piecewise 표현 거절 | 통과하면 `radius=offset`; 최대로 가능한 반경을 새로 설계하지 않음 |
| `local_path.py:304/336` | piecewise 실패 시 기존 3차 곡선 적합으로 fallback | fallback은 `minimum_radius`를 사용하지 않음. 적합 잔차/접힘 검사는 곡률 제한과 다름 |
| `local_follow.py:429–455` | 목표 곡률 `2*y/(x*x+y*y)`→`atan(wheelbase*kappa)`→±상한 clamp | lookahead `.30+.12*abs(speed)`, 범위 .30–.60 m. 상한/R_min과 무관 |
| `local_follow.py:1205–1222` | clamp한 목표를 3 rad/s 제한, 현재/직전 조향 swept clearance 검사 | 실제 이번 명령 원호가 원시 점유와 간섭하면 감속/정지. 이미 생성한 경로로 자동 조향 복구하지 않음 |
| `local_follow.py:449–454` | 경로 곡률·목표 곡률에 횡가속도/제동 속도 cap | 기하적으로 불가능한 곡률을 가능한 곡선으로 바꾸지는 않음 |
| `corner_boundary_support.py:126`, `current_wall_reacquire.py:40–49` | 같은 최소 반경과 같은 `pursue_path` 재사용 | 현재 짧은 경로 재획득도 같은 한계 공유 |
| `corner_corridor.py:50/195` | 일반 `local_path`로 양측/짧은 경로 근거 비교 | 이 표현 자체에는 최소 반경 전달이 없음; 최종 추종/점유 검사가 뒤따름 |
| `turn_preview.py:8–28/53` | 예측 중 같은 상한·3 rad/s 사용, 최고 곡률로 샘플 간 외곽 팽창 | 이번 기준 설정에서 `preview_turn_speed=false`; 활성화해도 주 목적은 추가 관측 거리의 속도 허용 |
| `solo_corner_approach.py:24–38` | 관측 횡단면까지 거리로 고속 감속 | 이번 기준에서 false. 켜도 v>1.2와 추가 solo flags 필요, `R_min` 대신 offset 사용 |
| `race_runtime_policy/corner_candidate.py:16/34` | 상한 검증 후 기존 조향 그대로 .1 m/s 거리 예산 제안 | `r396_v1` 이름의 corner는 조기 회전기나 경로 곡률 수정기가 아님 |

경로 생성이 바뀌지 않는 경우에도 더 작은 조향 clamp가 경로 이탈→현재 swept clearance 감소→속도 손실로 이어질 가능성은 있다. 실제 r544 자료에서 확인하기 전에는 가설이다.

## r544 결과를 받으면 분리할 항목과 최소 선택지

1. **원시 실제 점유에 막혔는지 먼저 구분한다.** 같은 시각의 `normal.reason`, `boundary_selection`, `wall_model`, `path_points`, `boundary_from_memory`, `swept_clearance_m`, `target_speed_before_hold_mps`, `current_rebase.output`, guard reason, 실제/요청 조향·속도를 맞춘다. 정답 progress는 실패 위치 분류에만 쓰며 제어 입력으로 쓰지 않는다.
2. **경로 자체 vs 추종 목표를 구분한다.** 저장 `path_points`와 당시 속도·직전 조향으로 `pursue_path`의 최근접점/목표점을 그대로 재계산한다. `unclipped_delta`, `clipped_delta`, rate-limit 이후 각, `kappa_requested/(tan(limit)/wheelbase)`와 전방 경로 곡률을 비교한다. 경로 곡률은 표본 차분 지표여서 한 점 최대만으로 불가능 판정을 추가하지 않는다. 현재 LocalFollow 세부 출력은 clamp 전 각과 lookahead를 모두 남기지 않으므로 **저장 경로에서 재계산하는 것이 우선**이다.
3. **경로는 완만한데 복귀 조향만 포화된다면:** 현재 관측 경로 안의 목표 선택/회전 선행 문제를 먼저 검토한다. 최소 후보는 같은 경로에서 더 앞의 실제 목표를 선택해 제한 안의 조향을 제안하고 기존 현재/직전 조향 점유 검사를 그대로 적용하는 것이다. lookahead 일괄 증대는 안쪽 모서리를 잘라 들어갈 수 있으므로 채택 근거가 아니다. 현재 raw에서 원호 외곽이 확인되는 후보만 비교한다.
4. **경로의 연속 구간 자체가 최대 곡률보다 급하다면:** 속도만 조정하지 않는다. `wall_model`이 일반 곡선 fallback인지 확인하고, 이미 있는 두 관측면/직선+원호 생성기를 이용해 허용 반경으로 접속할 여지가 있는지 검토한다. 현재 `corner_path`의 radius와 offset 분리는 가능한 국소 변경점이나, 원호 시작점이 앞당겨지고 양측 공간 여유가 바뀌므로 실제 반환 근거가 필요하다. `radius=max(offset,R_min)` 한 줄만 넣고 통과시키는 것은 충분하지 않다. 새 경로를 만들 수 없다는 이유만으로 추가 영구 정지 조건을 만들지 않는다.
5. **경로와 최종 각은 가능하지만 변화율·취득 지연 중 외곽이 닿는다면:** 조기 회전 또는 코너 전의 한정 속도 조정을 비교한다. 감속은 시간 지연 문제를 줄이는 선택지이고 물리 반경을 고치는 선택지가 아니다. 기존 `solo_corner_approach`를 일괄 켜는 것은 이번 정상 경로와 활성 조건이 다르므로 최소 수정이라고 볼 수 없다.
6. **경로/현재 원호는 가능한데 관측 지원 또는 오래된 기억만 0/.1을 만드는 경우:** 이는 조향 생성과 별개다. 해당 guard/current route 근거를 유지한 채 기존 r538 계열 관련성 검사에서 분리한다. 조향 상한이나 실제 점유 조건을 완화하지 않는다.

현 단계에서 독립적으로 확실한 성능 개선 코드는 없어 후보 제어기를 만들거나 채택하지 않았다. 실제 실패가 있으면 위 분기로 가장 작은 변경 하나만 비교하는 편이 원인 분리에 유리하다.

## 입력/검증 경계

숫자는 `numerics.json`, 검토 소스 SHA는 `source_manifest.json`에 남겼다. 실제 r544 trace header는 아직 확인하지 않았다. r544의 기존 `expected_header.json`에는 과거 max_steering_rad=0.4578436017036438이 남아 있으므로 정상적인 새 상한 적용도 기존 전체 header 비교에서는 차이가 날 수 있음을 root에게 전달했다. 이 파일의 다른 설정들은 기준 구성 확인에 사용했으며 실제 실행값으로 단정하지 않았다.
