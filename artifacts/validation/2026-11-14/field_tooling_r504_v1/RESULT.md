# r504 실장 조향 CLI 선택 범위 전달

2026-11-14. **실장 조향의 좁은 선택 범위를 기존 `goal` CLI에 전달할 수 없던 결함 1개를 수정했다.** Windows 파일·합성 입력만 사용했다. 센서/ESP32/구동/SSH/WSL/flash는 실행하지 않았고 r500 설치본은 변경하지 않았다.

## 확인한 결함과 수정

`runtime_installed_profile.py`는 선택 `servo.command_limit/wheelbase/track/wheel_limit`을 기존 backend에 전달한다. 반면 `steering_zero_profile.py goal`은 그 인자를 받을 수 없어 전체 CAD 범위를 항상 요구했다. 합성 중립2000·장착범위1880~2120에서 backend에 선택 상한0.15rad·바퀴각 한계0.2rad를 전달하면 중립 계산이 가능하지만, 원 CLI는 중립조차 `required range crosses mounted or count limit`로 exit2였다. [원 재현](before.json)은 보존했다. 이 숫자는 합성 재현값이며 실제 영점/장착 한계가 아니다.

`goal`에 `--command-limit-rad`, `--wheelbase-m`, `--track-m`, `--wheel-limit-rad`를 추가하여 **명시된 값만** 동일 backend에 전달한다. 생략 시 기존 CAD 기본값을 유지한다. 입력 프로필의 중립·방향·count 한계를 수정하거나 범위를 자동 확대하지 않는다. [수정 후 같은 중립 계산](after.json)은 exit0·2000counts·`output_enabled=false`다.

실제 확인한 설치 board의 `servo` 항목을 그대로 전달하는 사용 형태:

```bash
python3 scripts/steering_zero_profile.py goal "$ZERO_MOUNTED" \
  --steering-deg "$REQUESTED_CENTRAL_DEG" \
  --command-limit-rad "$BOARD_COMMAND_LIMIT_RAD" \
  --wheelbase-m "$BOARD_WHEELBASE_M" --track-m "$BOARD_TRACK_M" \
  --wheel-limit-rad "$BOARD_WHEEL_LIMIT_RAD"
```

이 명령은 목표 count 계산만 하며 장치를 열거나 출력하지 않는다. 설치 profile 생성·무결성 검증·선택 빌드 경로는 그대로다.

## 실측 입력 경로 확인

- r500 결과와 r491 전달 문서는 r401을 참고용으로 보존하며 실제 운용 프로필로 선택하지 않는다. 중립은 `record`의 필수 명시값이고, 방향/장착 한계는 `confirm-mount`의 필수 명시값이다. 미확인 상태의 영점은 설치 헤더로 내보낼 수 없다.
- board JSON은 encoder 방향·모터 방향 확인·PI/duty·명령 상한·servo 설정을 명시적으로 받는다. 생성기는 합성 fixture나 r401에서 누락값을 채우지 않는다. 생성 성공은 C++ 유효성/실장 검증이 아니며 manifest의 `cpp_valid_checked=false`를 유지한다. 실제 board 유효성은 기존 빌드의 `RuntimeBoardProfile.valid()`에서 확인한다.
- 선택형 `max_wheel_mps` 생략 시 기존5m/s가 유지되는 r495 계약도 그대로다. 이것을 사용자13km/h 실물 상한의 적용 증거로 간주하지 않는다. 실제 값·설치·물리 과도 검증은 조립 후다.

## 검증과 이관

새 CLI 검사5개 메서드/13대표 호출과 기존 조향 backend18개 메서드, 총23개 고유 검사를 확인했다. 최초22통과/1환경 오류(`arena_autonomy` 소스 경로 누락)를 [원 로그](unittest.stderr)에 보존했다. 패키지 경로만 추가하여 실패한 검사1개만 [재실행 통과](dependency_retry.stderr)했다. 기본 Python의 pytest 미설치 때문에 기존 표준 `unittest`로 실행했으며 의존성을 설치하지 않았다.

양 방향의 중립/선택 끝점, 선택 밖 거절, 미선택 시 기존 전체범위 요구, 기존 전체범위 호출, 잘못된 기하/비유한값 거절을 확인했다. 모든 CLI 호출 전후 입력 JSON SHA가 같고 결과는 출력 비활성이다. [수정 파일 SHA와 범위](verification.json).

수정 파일은 `scripts/steering_zero_profile.py`와 `tests/test_steering_zero_profile_cli.py`뿐이다. 공통 제어·실장 문서·mounted-servo 별도 작업·설치값은 수정하지 않았다. Jetson에서 새 옵션이 필요할 때만 검토한 스크립트를 이관하면 되며, 이 결과를 실제 장착 영점/방향/범위 채택이나 주행 검증으로 확대하지 않는다.
