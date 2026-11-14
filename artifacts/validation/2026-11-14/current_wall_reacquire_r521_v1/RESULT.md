# r521 현재 관측 저속 재획득 stage

공통 제어기·설정·WSL·Jetson은 변경하지 않았다. 작성 범위는 이 폴더다. 기존 현재 경로 함수를 재사용하는 stage 후보이며 실차/폐루프 주행 증명이 아니다. 실제 5T 벽의 굴곡·접합 틈·변위는 아직 수치가 없으므로 아래 합성 치수는 **원인 분리를 위한 임의 시험값**이다.

## 결론

과거 경로의 anchor/벽 일치만 실패했을 때, 현재 `local_path`의 짧은 경로를 기존 `pursue_path`와 **현재·직전 조향 양쪽 `swept_limit`**, 원시 점유, per-ray padding, marker clearance, 제동/추종 간격에 통과시키면 저속 양수를 낼 수 있다. `.10 m/s`는 stage 비교 상한이며 최종 기본값 결정이 아니다.

`LocalFollow` 단독 후보에 그치지 않고 `MovingPassAdapter.command`가 active 상태에서 `selected=None` 때문에 0을 반환하는 분기까지 같은 helper를 연결했다. 기존 정상 양수, 실제 점유로 거절한 후보, 입력/시간 오류에는 적용하지 않는다. 새 실행 프로필이나 별도 경로 탐색 체계를 추가하지 않았다.

기억벽이 영구히 달라도 기존 `handoff_follow`와 `handoff_monitor`를 새 현재 센서로 준비하면, 정상 현재 LocalFollow 경로가 생긴 뒤 기존 `owner.reset()` → 준비 객체 복원 → `_base_command()` 방식으로 FOLLOW에 넘길 수 있다. 인계만 해서 미래 우회 예약 때문에 0으로 떨어지는 첫 stage 결함은 수정했다. 기존 `_follow_entry_space_current`의 읽기 전용 결과를 준비 monitor로 확인하여 미래 reserve만 0이면 reset을 미루고 **현재 경로·점유·제동을 다시 통과한 저속 후보를 유지**한다. 실제 충돌 검사 0을 우회하지 않는다.

## 근거

[`focused_final.log`](focused_final.log) / [`focused_final.xml`](focused_final.xml): 10 focused 검사 통과. [`diagnostic.json`](diagnostic.json)은 출력·경로·점유·padding·source hash를 포함한다.

| 대표 입력 | 기존 출력 | stage 출력 | 의미 |
|---|---:|---:|---|
| 현재 벽 중심 +.16 m, anchor .160 m | 0 | .10 m/s | 기존 anchor .12 m 초과만으로 현재 경로를 버리지 않음 |
| 짧은 굴곡, 현재 경계 앞 길이 .5235 m | 0 | .10 m/s | 기존 .65 m 길이 미만이어도 관측된 경로 예산/제동으로 제한 |
| 같은 입력에 실제 전방 x=.18 m 반환 추가 | 0 | 0 | 점유를 지우거나 UNKNOWN으로 바꾸지 않음 |
| 실제 adapter/guide, 기억벽 -.20 m, 새 벽 +.16→+.08 m, actor 중심1.65 m | 매번 `current_outer_wall_unavailable` 0 | .10×4회 → FOLLOW .35 m/s | 기억벽 불일치 지속 중에도 새 정상 경로 인계 후 임시 cap 제거 |
| 위 조건, actor 중심1.2 m | 같은 과거 벽 실패 0 | .10×6회 유지 | LocalFollow 자체 .55, 미래 entry reserve1.2 m의0을 인계에 앞서 구분. reset만 미루고 현재 양수 유지 |
| 위 가까운 actor 시험 뒤 관측 간격 증가 | 기존 과거 벽 실패 | FOLLOW 양수, .10 초과 | 기존 준비 객체를 그대로 이어받아 임시 cap 제거 |

합성 입력은 500개 원각도 ray와 벽/actor 선분의 교차거리로 생성한다. source 시각과 raw scan은 scene과 일치하며, 비반환은 `inf`/UNKNOWN 그대로다. actor·장애물 반환을 유지한다. 연속 표는 센서/상태 전이 시험으로, 벽 상대 위치가 변한 이유를 자차의 폐루프 추종 운동으로 입증하지 않는다.

`.05/.10/.20 m/s` 민감도에서는 앞의 두 비점유 대표가 각각 지정 상한을 냈고 실제 장애물 대표는 모두 0이었다. 이를 더 높은 실제 속도의 근거로 확대하지 않는다.

### 저장 입력 3개

- r513 seq95: helper .10 m/s. 당시 원 MOVING_PASS는 .885597 m/s이므로 **연결안은 원 양수를 유지**한다. 개선 표본으로 세지 않는다.
- r513 seq110: helper .10 m/s, 당시 원 `entry_guide_current_wall_too_short` 0. 이 과거 원인은 root의 r514에서 이미 다뤘으므로 이번 stage의 현재 공통 개선으로 소급하지 않는다.
- r515 seq219: helper 후보 없음. 원 `no_entry_command_candidate` 0을 유지한다. 실제 점유 정체를 기억벽 문제로 바꾸어 분류하지 않는다.

각 trace는 SHA-256과 바이트 불변을 확인했다. 전체 trace 재회귀를 반복하지 않고 선택한 현재 센서 3개만 평가했다.

### 실행 전달과 남은 경계

원 MOVING_PASS도 `runtime supported()`의 보관/rebase 대상이 아니다. 기존 `RuntimeDualDelivery.poll()`의 비보관 plan 분기는 원 source<.3 s, command<.2 s, epoch·직전조향·벽측 및 양수 적용 간격을 검사한다. 메모리 worker fixture에서 stage 양수의 fresh 전달, source 만료 폐기, 직전조향 변경 폐기를 확인했다. 실제 ROS 발행/guard/MCU 시험은 아니다.

따라서 새 proof를 만들어 허가를 위조하지 않는다. 정상 FOLLOW 인계 후에는 현재 공통 `current_route_priority`/r508 경로를 그대로 사용한다. STOP/RESET의 기존 invalidation 및 독립 `/drive`→guard→`/drive/safe` 경계는 변경하지 않았다.

정면 UNKNOWN이 넓은 합성에서는 기존 partial hold가 LocalFollow의 `release_count`를 계속 초기화하여, 현재 긴 벽이 확보된 뒤에도 정상 인계가 지연되고 임시 저속만 반복될 수 있다. `open_front_existing_partial_limit`에 별도 보존했다. 이것은 stage가 새로 추가한 정지 조건이 아니라 기존 경계이며, 이번 stage에서는 이를 고치거나 묵인하지 않았다.

## 최소 통합안

구현은 [`stage/current_wall_reacquire.py`](stage/current_wall_reacquire.py)에 있다. 수용 여부는 root가 선택한다.

1. `current_wall_candidate`를 순수 공통 helper로 옮긴다. 경로 생성은 기존 `local_path`, 추종/속도/점유는 기존 함수다. 관측점 필터/새 상태기계/후진은 없다.
2. LocalFollow의 `follow_boundary_unconfirmed` 0 분기에서만 helper를 호출한다. 정상 출력과 actual occupancy 0은 그대로 둔다. stage의 `CurrentWallReacquire` subclass가 이 범위다.
3. MovingPassAdapter의 active `chosen is None` 분기에서, 현재-기억 벽/경로 불일치 이유에 한하여 **같은 helper**를 호출한다. 공통 통합 때 `_prepare_follow_handoff`의 기존 객체 준비 부분을 공유하면 stage의 짧은 초기화 중복을 없앨 수 있다.
4. 새 LocalFollow가 현재 source의 정상 양수를 내면 준비 monitor로 기존 `_follow_entry_space_current`를 읽는다. 미래 reserve만0인 경우에는 인계/reset을 미루고 같은 helper를 재검사한다. 그 외에는 기존 centered 인계 절차를 사용하고 상위 `_base_command` 결과를 존중한다. monitor 참조만 `try/finally`로 잠시 바꾸며 전체 owner deepcopy/상태 복원은 없다. 현재 범위 밖의 다른 상위 정책까지 일반화한 인계 원자성 증명은 아니다.

```text
normal = existing moving-pass result
if normal positive or failure is not current/remembered geometry mismatch:
    return normal
prepare existing handoff_follow/monitor from this current scan
if prepared current LocalFollow positive and existing future-reserve preview is not zero:
    reset old pass; restore prepared follow/monitor
    return existing owner base-command result
candidate = current_wall_candidate(same current raw points, actual previous steering)
return candidate through existing plan delivery if candidate exists else normal
```

최종 선택에 필요한 남은 쟁점은 새 filter가 아니라 **현재 관측 경로 재개 시 partial hold가 실제로 남기는 주행 손실과 저속 구간 길이**다. 미래 reserve를 전진 필수 거리로 다시 요구하지 않는다. r515의 실제 점유 정체를 해결했다고 주장하지 않는다.

## 보존과 재현

- 기준 wall-fit SHA: `c95440591c58fcb9ac058ba8d59b45d6419a12ea42a7148abf2c108b4d8c370e` (r518 통합).
- `diagnostic.json.source_sha256`에는 r508 이후 current-route 파일을 포함한 실제 읽은 공통 소스 hash가 있다. 해당 진단 구간에서 모두 불변이었다.
- Windows Python 3.14/NumPy 2.5 환경의 2D `np.cross` 제거는 시험 프로세스 안에서 기존 determinant로만 호환했다. 공통 코드/설치 패키지는 변경하지 않았다.
- `focused_v1`~`focused_v8`은 fixture/import/시각 정합/정상복귀 조건 및 tracker 초기화 과정의 실패를 보존한다. 초기 실패를 최종 성공으로 덮어쓰지 않았다.
- `diagnostic_before_reserve_defer.json`에는 첫 stage의 가까운 actor 인계 .10→0을 보존했다. 최종 결과는 미래 예약 때문에만 인계를 서두르는 문제를 고친 뒤 다시 실행한 별도 값이다.
- 재현은 저장된 r429 `pydeps`를 `PYTHONPATH`에 두고 `python -m pytest .../test_stage.py`, `python .../diagnostic.py`를 실행한다. 실제 차량·ROS·WSL을 실행하지 않는다.
