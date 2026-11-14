# r550 25.81 m 정체의 독립 기하 분석

공통 소스·WSL·장치 변경/실행 없음. r550 `stall_extract`의 `last_positive_current_input`과 `first_zero_candidate_current_input`을 Windows에서 같은 MotionHistory로 복원하고, 경로 fit 없이 실제 반환점의 sweep/제동 수식만 평가했다. NumPy 2D cross 대체나 전체 제어기 재생은 하지 않았다.

**직접 원인은 원시점의 실제 차체 접촉이 아니라 취득 후 이동 상한을 양축 각각 더한 사각 모서리의 접촉이다.** 첫 정지의 pursue는 이미 유효하며 anchor 오차 .2489 m가 이번 helper의 정지 조건은 아니다.

| 항목 | last positive, 61.12 s | first zero, 61.22 s |
|---|---:|---:|
| 실제 반환 수 | 477 | 477 |
| 조향 현재/직전 | −.28475436568 rad | 동일 |
| 실제 속도 | .122450 m/s | .061225 m/s |
| raw margin 0 sweep | 1.12 m | 1.12 m |
| 고정 .025+.03=.055 m 여유 sweep | 1.04 m | 1.04 m |
| 기존 고정+점별 .5×age 사각 여유 sweep | 1.04 m | **.04 m** |
| 고정 .055 사각+점별 .5×age 원판 수식 대조 | 1.04 m | **1.00 m** |

원판 대조도 원본과 같은 .04 m 표본 및 한 칸 차감을 적용했다. first zero에서 `additional_clearance`는 .04 m를 더 줄이지 않는다. 이후 `available=max(0,.04-.16)=0`, `braking_speed(0,age)=0`으로 helper가 None이 된다. .16 m 정지 간격과 braking 내부 .06 m를 임의 축소할 필요 없이 그 전의 기하 원인을 분리할 수 있다.

## 실제 검출점과 이동 반경

첫 간섭 표본은 후륜축 이동 .08 m이다. 고정 여유가 포함된 직사각형 반길이는 .1585 m, 반폭은 .130 m이다.

| source | 현재 반환점 (x,y) m | age s | 고정 상자까지 필요한 이동 (x,y) mm | 필요한 norm mm | 허용 .5×age 반경 mm |
|---|---|---:|---|---:|---:|
| 224 | (.296179,.109946) | .072906 | (32.112,36.339) | **48.494** | **36.453** |
| 225 | (.299478,.107839) | .072709 | (35.740,34.867) | **49.930** | **36.354** |

source224의 회전 차체 기준 y=.166339 m는 기존 사각 한계 .075+.091453=.166453 m에 0.114 mm 들어간다. 하지만 고정 상자까지 48.494 mm를 이동해야 하므로, 속력 상한을 2D norm 상계로 해석하면 허용 반경보다 12.042 mm 부족하다. source225도 13.576 mm 부족하다.

고정 .055 m 사각 여유를 그대로 두고 이동 불확실성만 원판으로 더한 식은 다음과 같다. 점·age·속력 bound·제동/정지 간격은 모두 원래 값이다.

```text
dx = max(abs(local_x) - (length/2 + .055), 0)
dy = max(abs(local_y) - (width /2 + .055), 0)
hit = hypot(dx, dy) <= .5 * (reference_s - acquired_at_s)
```

전체 477점을 1 mm 간격으로 독립 대조한 첫 .30 m 궤적의 최소 여유는 **+9.118 mm**이다(source221, 이동 .084 m, 필요한 norm45.866 mm/반경36.748 mm). 같은 대조의 최초 먼 접촉은 1.036 m다. 이는 수식 비교이며 실제 폐루프 주행 보장이 아니다.

명확한 실제 전방 반환 `(.18,0)` 하나를 추가한 반례에서는 −L 및 직진 모두 원판 sweep=0, available=0, target=0을 유지한다. 기존 반환을 제거하지 않았으며 UNKNOWN을 자유 공간으로 만들지 않았다.

## 연결 범위와 일관성

- `current_wall_reacquire.py:57~65`와 `local_follow.py:1210~1225`가 동일한 고정+운동 사각 padding을 현재/직전 조향에 적용한다. 원판 해석을 선택한다면 두 조향 모두 같은 방식으로 유지해야 한다.
- `current_path_recheck/_geometry.py:67` 및 `race_runtime_policy/retained_recheck/_geometry.py:68`의 후속 재검사도 같은 사각 padding을 쓴다. helper만 바꾸면 후속 경로에서 다시 0이 될 수 있다. `corridor_observer.py:104`의 현재 corridor 후보도 같은 해석이므로 적용 범위를 명시해야 한다.
- 독립 `lidar_safety.py:271`의 최종 raw 검사에는 현재 기본 고정 margin=.025만 전달된다. 저장 첫 정지의 이 기하 결과는 **1.08 m**이며 위 운동 사각 과대 팽창을 반복하지 않는다. 독립 입력·명령 제한·STOP/미관측 정책은 별도 유지한다.
- `guard_memory_command.py`의 현재/직전 명령 외곽도 기본 고정 margin을 쓴다. 소실점 재관측의 2 m/s 도달 영역은 별도 증거 계약이므로 이번 원인 수정과 섞지 않는다.
- 선택 영상 점유 `vehicle_marker_occupancy.py`는 별도 .5×age 및 샘플 양자화 padding을 더한다. 활성화했을 때는 이 경로의 해석 차이도 남을 수 있다. 이번 계산은 영상 보조의 렌더·실행 검증이 아니다.

최소 제안은 공유 `swept_limit`의 기존 기본 연산 경로를 유지하며 **고정 사각 margin과 선택형 원판 이동 반경을 분리**하는 것이다. `per_ray_padding(scene, 0, bound)`를 반경으로 계산하면 기존 age/유한성 검증을 재사용할 수 있다. 현 `per_ray_padding` 반환값 전체를 전역적으로 바꾸면 다른 경로·점유 함수들의 축별 padding 계약까지 바뀌므로 피한다. 구현·적용 범위와 최종 채택은 root가 결정한다.
