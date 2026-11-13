# r489 독립 guard 현재 관측 정책 — 공통 이관·연결 확인

root가 검토 후 공통 6파일을 이관했습니다(`integration_v1/applied.json`). `selected_r181`과 `traffic_recovery`가 명시된 독립 guard만 현재 관측·명령의 실제 점유/제동을 우선하도록 정리했습니다. Windows 비ROS 검사 33개와 8 subtests 및 후속 WSL 관련 검사 180개가 통과했습니다. 실제 ROS 생성자의 선택 연결도 확인했고 spin/control/구동 발행은 하지 않았습니다. Jetson·센서·실차·새 주행 검증은 없으며 자기 소유 잔존 PID는 없습니다.

## 변경

- `lidar_safety.py`: 정책 선택값을 선언하고 명시 조합에서만 새 분기를 연결합니다. 기존 현재 조향 `swept_limit` 한 번과 제동식을 유지합니다. 직전 조향을 전체 제동 곡선에 추가하는 초기 stage 변경은 제거했습니다. 직전 조향은 기존 실제 소실점 위협 검사에 그대로 남습니다.
- `independent_guard_route.py`: 기존 `on_scan → mask_scan` 이후 CAD 거리 band 안의 자기 반환이 제외된 입력에서 현재 외부 전방 반환의 존재를 검사합니다. 전방 전체 소실·최소거리 미만·각도 메타데이터 오류를 거절합니다. 자기 각도 sector 전체를 다시 제거하지 않아 같은 각도의 먼 외부 반환은 유지합니다. 거리/자유 공간은 생성하지 않습니다.
- `partial_observation.py`: 기존 실제 소실점 검사·재관측·래치·기억 갱신 본문을 공유 helper로 추출했습니다. 새 명시 분기는 전역 결측 각도/chord/시험속도상한/과거 자유 관측 warmup을 사용하지 않습니다. 실제 남은 기억은 삭제하지 않고, 운동 이력으로 옮겨 기존 위협 검사에 넣습니다. 없는 과거 관측을 가상의 장애물로 만들지 않습니다.
- `known_structure.py`: 선택 guard에서 가상 먼 그림자의 cap만 비활성화합니다. 실제 `structure_hidden` 갱신과 제동 cap은 유지하며 선택모드 진단에 두 사실을 명시합니다. default 반환 필드는 변하지 않습니다.
- `runtime_installed_limits.py`: 원본 r401 계열 profile은 planner 아래에만 선택값을 갖습니다. 새 설치 profile을 파생할 때 planner `selected_r181` 및 양쪽 `g1_traffic_recovery_trial=true`가 모두 명시되면 guard에도 선택값을 씁니다. 기존 YAML은 덮지 않으며 명령/허가/경로를 guard로 전달하지 않습니다.
- 신규 `tests/test_independent_guard_route.py`: 실제 `control` 본문을 AST로 읽어 비ROS 메시지/시계 대역에서 실행합니다. 보호 판정 함수를 복제하지 않습니다. ROS node 생성·토픽 배선·타이머는 이번 검사 범위 밖입니다.

## 확인 결과

`windows_tests_final3.log`: 신규22 + 기존 설치상한11 = **33 passed, 8 subtests passed**. 자기 반환만/후방만 거절, 동일 각도 먼 반환 유지, 전방 전체 소실/최소범위 미만/메타데이터 오류, 넓은 측방 결측의 실제 반환, 실제 근접점 소실 래치, 기존 두 fresh 완전 관측 회복, timed Recovery의 실제 투영·소실 기억, 실제 hidden memory cap, STOP·명령/스캔 만료·역방향/NaN·운동 실패·직접 점유 제동, 현재 조향 sweep 1회 및 등록 frame 계약, profile 선택 연결을 확인했습니다.

`default_exact.json`: 동결 원본과 후보의 default 출력·내부 기억 상태를 **15단계 수치 허용오차 0**으로 비교했습니다. 기존 `follow_trace`에서 비결정적 진단으로 분리하는 `surface_correspondence.processing_ms`와 그 상태 cache 위치만 비교에서 제외했습니다. 해당 raw 출력의 차이를 보존했으며, 이를 전체 raw 바이트 동일로 주장하지 않습니다. fully observed, partial warmup, 실제 소실 래치/회복, 재관측, 기존 속도 거절, 구조 숨은 점 기억을 포함합니다.

`saved_input_comparison.json`: r485 원 sensor/motion 입력·실제 기록 명령의 4대표를 사용했습니다. 실제 guard 이전 safe 조향/내부 기억/타이머 순서가 없으므로 cold 정책 비교이며, 저장 planner 이전 조향을 대체 관측값으로 명시했습니다. 원 입력·명령 파일 해시 불변을 확인했습니다.

| 원 seq / reference | 기존 사유 | 후보 사유 | 계산된 속도 기존→후보 |
|---|---|---|---|
| 2893 / 49.92 s | fully_observed | fully_observed | 1.0→1.0 m/s |
| 2905 / 50.12 s | fully_observed | fully_observed | 1.0→1.0 m/s |
| 2911 / 50.22 s | lateral_metric_outside_trial | current_command_observed_returns | 0→0.85 m/s |
| 2916 / 50.32 s | lateral_metric_outside_trial | current_command_observed_returns | 0→0.85 m/s |

위 속도는 현재 반환에 대한 기존 sweep/제동과 관측된 명령으로 계산한 계약 결과입니다. 새 `/drive/safe`를 발행한 결과나 전체 guard exact 재현, r485 실패의 소급 성공이 아닙니다. 원 r485의 `safety_clearance_m=0`은 partial 거절 뒤의 초기값으로, 실제 충돌거리 0이 아니었습니다.

## 수용 범위와 남는 위험

독립성은 `/drive → guard → /drive/safe`와 guard 자체 센서·운동 이력에 유지됩니다. planner 경로·허가, 지도, Gazebo 정답을 입력받지 않습니다. 최소 유효 수/신선도·명령 무결성·STOP·현재 직접 점유·실제 알려진 소실물체 기억은 남습니다. 일부 반환 존재는 전체 전방의 자유 공간 증명이 아니며, 이전에 관측되지 않은 숨은 물체·희소 반환 사이의 물체·상대 운동의 모든 경우를 보장하지 않습니다. 실제 조향 지연의 새 swept 형상은 이번 변경에 추가하지 않았습니다.

첫 Windows 시도는 기존 PyYAML 경로 누락, 다음은 기존 테스트의 common module 선점, 다음은 오류 문구 정규식 차이로 실패했습니다. 기존 설치 모듈 경로/선행 stage import/기존 문구 보존으로 수정했고 원 로그를 남겼습니다. default 저장기는 frozenset 직렬화와 표면정합 wall-time 비교를 보완했습니다. 추가 timed Recovery 소실 대표의 첫 fixture는 기억에 1점만 남아 기존 surface anchor의 최소3점 계약을 충족하지 못했으므로 `traffic_recovery_invalid`로 정지했습니다. 이 실패를 보존하고 정상 관측 배경이 있는 대표로 수정한 후 실제 소실 래치 및 기억 갱신을 확인했습니다. 정책 판정선을 완화하거나 운영 코드를 이 실패에 맞춰 고치지 않았습니다.

## 검토·통합

`source_manifest.json`에 변경 6파일의 before/candidate SHA256과 공통 원본 일치를 기록했고 `candidate.patch`를 준비했습니다. `prepare_stage.py`는 초기 stage 생성용이므로 지금 재실행하면 검토 반영분을 덮습니다. 공통 이관은 root 소유입니다.

```powershell
python artifacts/validation/2026-11-14/independent_guard_route_r489_v1/run_windows.py
python artifacts/validation/2026-11-14/independent_guard_route_r489_v1/verify_saved.py
git apply --check artifacts/validation/2026-11-14/independent_guard_route_r489_v1/candidate.patch
# root의 검토·소스 beforehash 재대조 후에만:
git apply artifacts/validation/2026-11-14/independent_guard_route_r489_v1/candidate.patch
```

위는 Windows stage 검토 시점의 명령과 근거입니다. 이후 common 통합 및 최소 WSL 연결 확인은 아래에 기록합니다. native 성능 행렬은 시행하지 않았습니다.

## 공통 이관 후 WSL 연결 결과

위 검토 요청 이후 root가 공통 6파일을 이관하고 지정한 관련 6개 테스트 파일 및 ROS 생성자 연결 확인을 수행했습니다. 기존 Jazzy와 메시지 설치를 재사용하며 common Python 소스를 직접 import하므로 재빌드는 하지 않았습니다. constructor는 실제 `LidarSafety`와 `RecoveryPolicy` 두 객체의 `command_observation_priority=true` 및 선택값 `selected_r181`을 확인하고 spin 없이 destroy/shutdown했습니다. [constructor 결과](wsl_verify_v1/constructor.json)에 근거가 있습니다.

첫 WSL 회귀는 100 passed / 80 failed였습니다. 전부 기존 `test_lidar_safety.node_double`에 marker OFF channel이 없는 같은 오류로, [원 결과](wsl_verify_v1/RESULT.md)와 로그를 보존했습니다. root 승인으로 공통 `tests/test_lidar_safety.py`의 그 대역 한 곳에 `profile=None`, `snapshot=None`, `reset=noop` 계약만 추가했습니다. 이를 가져오는 partial/known-structure fixture도 함께 수정 효과를 받았습니다. 생산 코드는 추가로 바꾸지 않았습니다.

동일 6파일 재실행 결과는 **180 passed, 3.00s, exit0**입니다. [재실행 로그](fixture_fix_v1/pytest.log)·[소스 불변/PID0 회수](fixture_fix_v1/collected.json)·[fixture 변경 영수증](fixture_fix_v1/receipt.json)을 남겼습니다. WSL은 반환했습니다. 실제 launch 전체/새 주행·실차 결과는 여전히 별도입니다.
