# RELEASED parallel의 현재 차체 옆 공백 비교 r375

## 확인한 원 실패와 범위

[r371 원 주행](../relative_body_cad_drive_r371_v1/README.md)은 10.13m 뒤 정체로 실패했다. 이번 작업은 저장 입력의 제한 후보이며 원 주행 결과를 변경하지 않는다.

원 `follow_decisions.jsonl` SHA256은 `1f2339aac85303432e07deee6932aa3340b4ef65b8eb963b2be2f4cc70bec910`이다. 저장 현재 검사 217개는 RELEASED 162·SOLO 43·GUARD 12개이며, `route_current_large_motion_region_unobserved` 정지는 93개다. 이 93개는 모두 RELEASED의 **parallel** 경로이고 기존 차체 공백 검사의 `near_gap_neighbor` 거절이다. baseline 확장은 이번 후보에 포함하지 않는다.

93개 원 계획 목표는 0.081463–0.091274m/s, 현재 실제 속도는 0–0.061225m/s이다. 마지막 목표는 0.08945631980895996m/s이며 현재 질의 584개 중 미관측 124개, 왼쪽 마스크 270개, 차체 옆 미관측 초과 5.5408mm, 마스크 꼬리 4.4101mm이다. 기존 고정 이웃 조건은 0.6m이다. 좁은 공백 속 숨은 물체가 없다는 증거로 해석하지 않는다.

## 명시 후보

[`candidate.py`](candidate.py)는 먼저 r370의 전체 현재 결과를 구한다. 이미 양수인 출력과 다른 정지는 그대로 반환한다. 추가 적용은 다음 범위다.

- `require_current_region=True`, 기존 `released_body_gap_trial`/`traffic_recovery_trial`이 켜진 FOLLOW/RELEASED/parallel.
- 원 목표 `0 < target <= 0.10m/s`, 현재 측정 `0 <= speed <= 0.15m/s`, 기존 CAD 조향 범위와 `0 < dt <= 0.1s`.
- 원 경계가 기억에서 오지 않았고 부분 관측 판정이 `fully_observed`이며, 같은 호출의 기존 고정 공백 검사가 `near_gap_neighbor` 하나로 거절한 경우.
- r370의 동일 `core_policy.rebase`를 **원 출력 그대로** 재호출한다. 원 RELEASED parallel의 선택 경로 일치, 직선 parallel 적용 범위, 10mm 횡이동 제한, 현재 전방 관측과 새 측방 물체 검사, 원 시각/epoch/경로 유효기간을 유지한다.
- 현재 실제속도·현재/직전 조향·원 목표로 전체 이동 영역을 다시 구하고, 원 거절 영역과 완전히 같은지 확인한다. 원 계획 목표를 0.10으로 올리거나 새 감속 목표를 탐색하지 않는다.
- 이 영역의 미관측 질의만 기존 `bounded_body_nonreturn(relative_neighbor=True)`로 검사한다. 모든 해당 질의·모든 광선 원점의 최대 반경에 최대 시간/거리/물체운동 padding을 더한 값이 이웃 거리 조건이다. 10mm 차체 옆 띠·6° 총 공백·5cm 국소 폭·전방 제외·원시 시각/거리 일치·현재 점유·마스크 꼬리 제한은 유지한다.

공통 `src/`, r370 이전 후보, 운영 기본값·TTL·차체 외곽·물성은 수정하지 않았다. 별도 정책 파일을 명시해 호출할 때만 새 비교가 수행된다. 자유 공간을 생성하거나 `/drive/safe`를 발행하지 않는다.

## 저장 검증 결과

[`test_candidate.py`](test_candidate.py)와 기존 [`test_relative_body_nonreturn.py`](../../../../tests/test_relative_body_nonreturn.py)를 합쳐 **59검사 통과**했다. [실행 영수증](../receipt_verify_r375_r374_20261113_b/r375_tests.log)과 [저장 비교 결과](results_native_v1/comparison.json)를 별도로 검토했다. 두 명령 종료코드 0이며 실행 전 해시 항목은 모두 그대로다. 실행 뒤 해시 목록에 새 결과 파일이 추가된 것은 입력 변경과 구별했다.

native_v1에서 r371 기존 출력 217개를 모두 재현했다. 문제의 RELEASED parallel **93개만 0→원 목표 0.081463–0.091274m/s**로 바뀌었고 나머지 124개는 그대로다. r364 249개·과거 r279 52개·r282 54개도 r370 결과 전체가 같다. 원 통과 출력은 즉시 반환하므로 새 조건이 정상 명령을 다시 정지시키지 않는다.

마지막 입력은 0→0.08945631980895996m/s이며, 이웃 반환은 11.37966m/0.55589m였다. 현재 질의 도달 반경 85.308mm+최대 padding 115mm=**200.308mm**가 상대 거리 조건이다. 공백 각도 약 4.32°·국소 폭 15.099mm이며 미관측 124개/전체 질의 584개는 지우지 않았다. 원 조향·경로·시각·10mm 횡이동 제한을 유지했다.

같은 마지막 저장 입력을 한 조건씩 바꾼 **17반례는 전부 정지·기존 출력 동일**이다. 후방 공백의 근접 이웃과 새 전방/왼쪽 반환은 현재 점유 거리 검사, 새 오른쪽 반환은 RELEASED의 현재 측방 검사에서 거절됐다. NaN·최소거리 미만 유한값, 속도·시간·epoch·선택 경로·측방 이탈·옵션/기억 경계 반례도 새 허가를 얻지 않았다. 이는 계산 반례이지 물리 장면 주행 결과가 아니다.

[`compare.py`](compare.py)는 다음을 한 번에 확인한다.

1. r371 현재 217개 전체의 기존 출력 재현. ROS 부모가 붙인 감속 상한만 기록대로 복원한다.
2. 새 후보의 적용/미적용 구분, 원 목표/경로/조향/시각·미관측 수 보존.
3. r364 저장 현재 249개 및 과거 교통 106개에서 r370 결과 전체 불변.
4. 마지막 실제 저장 입력의 단일 변경 반례: 실제속도/원 목표/시간/epoch/선택 경로/측방 이탈, 공백 안 NaN·근거리 유한값, 후방 공백의 근접 이웃, 새 전방 장애물, 양쪽 측방 반환.
5. 입력 SHA256 불변, `sys.modules`에 등록된 소스 해시와 ZIP 보관. 동적 import로 만든 r375 후보/시험 원문은 이 ZIP에 자동 포함되지 않으므로 별도 실행 영수증의 전후 SHA256으로 대조했다. 후속 r377은 후보 파일을 명시적으로 archive에 포함한다. 기존 결과 폴더를 덮어쓰지 않는다.

재현은 기존 native 환경에서 저장소 루트를 cwd로 사용한다. `PYTHONPATH`는 기존 실행 환경에 맞춰 `src/arena_autonomy`와 `tests`를 포함하고 native 확장 위치를 유지한다.

```bash
python3 -m pytest -q artifacts/validation/2026-11-13/released_current_r375_v1/test_candidate.py tests/test_relative_body_nonreturn.py
python3 artifacts/validation/2026-11-13/released_current_r375_v1/compare.py --backend native_v1
```

결과를 덮어쓰지 않으며 재현이 필요하면 `--output-dir`에 새 폴더를 지정한다. 저장 입력 계산 비교는 새 Gazebo 주행·완주·Jetson 처리율·실차 안전·기본 운영 채택을 증명하지 않는다. 실제 정체 해소 확인을 위한 [r377 제한 주행 준비](../released_cad_drive_r377_v1/README.md)를 별도로 작성했다.
