# r486 현재 관측 경로로 추월을 조기에 중단하는 후보

주 담당이 생산 2파일을 공통에 이관했고, 실제 공통 import와 native backend에서 관련 **185개 회귀가 통과**했다. 실물 기본 활성화·장치 실행·새 교통 주행은 하지 않았다. 대형 trace가 필요 없는 신규 영구 회귀 4개는 `stage/tests/test_moving_pass_current_return.py`에서 별도 이관을 기다린다. 원 r478 실패를 보존하며, **정지 전 선택 개선 후보**로 분류한다. 저장 입력의 두 연속 호출에서는 중심 복귀 목표를 유지했지만, 이후 원 차량의 운동 입력에서는 다시 기존 관측 한계로 거절된다. 지속 복귀·실제 주행 성공으로 판정하지 않는다.

## 후속 공통/native 회귀

[주 담당 이관 영수증](integration_v1/applied.json)의 생산 SHA와 실제 import 경로를 먼저 검사했다. `moving_pass.py`와 `bypass_entry_trial.py`는 공통 `src/`에서 로드됐고, 검사 뒤에도 SHA가 같았다. 기존 `_numeric_kernels` SHA `1bbe89cdcff6662fda699847756ff9c776e1b70c0d406ec97df469666dc3b16c`를 사용해 `numeric_kernels.configure('native_v1')`를 명시했고 종료까지 유지했다. 새 빌드/설치는 없다.

`moving_pass` 62 + `medium` 15 + `faster` 15 + `entry` 48 + `entry-guide` 22 + 최종 stage 19 + 신규 영구 4 = **185 passed**, pytest 6.96s, runner 7.16s, exit0이다. 따라서 앞선 Windows의 v2/v3 분할 결과 외에도 최종 stage 19개 전체가 통합된 생산 코드에서 재확인됐다. [원 로그](native_v1/pytest.log), [XML](native_v1/pytest.xml), [import/SHA 영수증](native_v1/receipt.json).

신규 영구 테스트는 기존 `sensor_scene`와 `stationary_history` fixture만 사용한다. follow_headway를 이동영역 전에 적용하면서 실측 속도를 유지하는지, 대상 미관측/측방이면 복귀를 시작하지 않는지, 새 중심 경로가 후속 호출·센터 도달까지 유지되고 reset에서 지워지는지를 검사한다. 대형 r478 trace는 해당 파일의 의존성이 아니다. [추가 테스트 패치](portable_test.patch)와 [파일 SHA/실행 명세](portable_tests.json)를 별도로 보존했다.

소유 Python PID758은 정상 종료했고 `/proc/758` 없음·잔여0을 확인했다. WSL은 주 담당에게 반환했다. ROS는 기존 회귀의 import에만 사용했으며 노드 생성/spin/Gazebo/센서/Jetson/구동은 없었다. [자원 반환](native_v1/resource_return.txt).

## 원 실패와 복구 입구

r478 원 결과는 `passed=false`다. GREEN 7.401~17.401s의 10초 동안 2.35m 진행 후 정지했고, 앞차 이동 중 최장 정체는 5.262s였다. 접촉0, 최소 표본 간격 0.447899m, 최소 swept 하계 0.447305m, command_expired0, STOP 검사 전부 통과, 자식 14/14 정상 종료, 동결 1351입력 불변이다. 원 8초 정체 기준에 닿기 전에 10초 창이 끝나 조기 STOP은 호출하지 않았다. 원 `inside_road=false`는 기존 차체 기준이며, 현재 형상 바퀴 중심 평가에서는 최대 1개 이탈/3개 이상0이다. 상세 원 판정은 [r478 execution_summary](../traffic_current_route_r478_v1/execution_summary.json)와 [원 report](../traffic_current_route_r478_v1/run1/report.json)를 유지한다.

- 마지막 양수 plan seq560(12.12s) 이후 seq563(12.22s)부터 지속 0이다. 원 `moving_pass_current_path_unavailable`의 하위 이유는 `known_occupied_command_hold`다. 해당 명령의 clearance가 0.08→0.04m로 줄어 원 제동 공식의 목표가 0이 됐다. 대표 원 region은 `unobserved=0`, `masked_front_samples=0`이므로 최초 지속 정지를 unknown 관문 탓으로 분류하지 않는다.
- seq563의 차체 좌표 x=0.2858~0.2881m, y=0.0657~0.0782m인 반환(ray230..234)이 약 90.5mm 점유 padding을 포함한 새 조향/직전 조향의 0.08m 진행 표본에 걸린다. 원 센서와 공개 Python 기하 경로로 네 대표의 원 clearance가 일치했다. 평가 pose/STL 단면 대조는 old610 신호 시설 표면에서 약 0.08~0.29mm 거리였다. **이 평가 좌표/시설 식별은 원인 대조에만 사용했으며 stage 제어 입력에는 없다.** [원 기하](../traffic_current_route_r478_v1/analysis_v1/failure_geometry.json), [시설 위치 대조](../traffic_current_route_r478_v1/analysis_v1/facility_match.json).
- 정지 뒤 fresh LocalFollow 단일 3행과 동일 인스턴스 연속 5행은 모두 `follow_boundary_unconfirmed`, swept 호출0이었다. 후속 Windows 진단에서 선호/반대 벽 anchor error가 각각 0.24635674/0.24637617m로 원 허용 0.12m를 넘었다. 관측 전방 길이는 1.8/1.0741m로 원 최소 0.65m를 넘는다. 확인 횟수만 더 쌓으면 해결되는 상태가 아니며, 해당 0.12m를 늘리지 않았다. [anchor 진단](boundary_reacquisition_cause.json).

## 변경 내용과 현재 근거

`moving_pass.py`의 `MovingPassGuide`는 entry 상태에서 기존 추월 목표가 **현재 점유 clearance에 따른 제동**으로 제한될 때, 현재 재확인 outer wall을 이용한 offset=0 복귀 경로를 비교한다. 대상의 현재 반환이 있고 기존 `ahead_clear()`가 참이어야 비교한다. 이 판정은 객체 부재나 자유 공간을 의미하지 않는다. 기존 계산은 현재 연관 반환의 최소 x에서 차량 대각선 0.25m, 측정 padding, 취득 나이×이동 상한을 빼 대상 후단의 조건부 하계를 얻고, 자차 앞단+0.06m보다 앞인지 확인한다. seq524의 하계는 0.1817163m, 요구 하계는 0.1635m다. 물체 ID나 미래 가속 보장은 아니다.

`bypass_entry_trial.py`에 기본 false인 내부 `follow_headway` 선택을 추가했다. 새 복귀 후보는 기존 일반 추종의 standstill/headway 계산을 **motion-region 검사 전에** 적용한다. 원 측정 속도, 새 각/직전 각 swept, 점유 padding, 원 scan/epoch, 조향 slew, unknown/mask 판정은 유지한다. 양수 후보가 실제 검사에 통과한 뒤에만 guide path/offset/phase를 변경한다. 실패하면 원 proposal과 상태를 유지한다.

수락 후에는 동일 guide의 `rejoin` 경로와 `follow_headway`를 후속 호출에 유지한다. 기존 센터 도달 조건과 준비된 LocalFollow/monitor 객체 인계를 재사용한다. 센터 도달 전에 매번 fresh LocalFollow로 바꾸지 않는다. reset/source epoch 변화 시 새 상태도 지운다. 직전 조향을 무시하거나, 점유를 삭제하거나, heading 0.08rad 및 기타 판정선을 완화하지 않았다.

반환 proposal은 selected offset=0, candidate path=`guide.path`, 동일 현재 벽 reference로 구성된다. 옛 편이 candidate/path는 함께 반환하지 않는다. **기존 MOVING_PASS 출력은 `current_path_recheck.scope()` 대상이 아니다.** r478의 call126개는 plan이고 current0이었다. 이 stage는 처리 범위를 늘리지 않으며, 새 경로는 다음 plan의 guide에서 재검사한다. 센터 도달 뒤 기존 일반 추종 처리로 인계한다.

## 저장 입력 비교와 의미 있는 회귀

[최종 국소 비교](stage_probe_v2.json)는 원 guide 출력 상태/현재 tracker 관측에서 시작한 비교다. 원 스캔·운동·직전 조향을 그대로 사용하며 새 조향을 적용한 폐루프 재현이 아니다.

| seq / 시각 | 원 측정 속도 | 후보 결과 |
|---|---:|---|
| 524 / 10.92s | 0.9183756302m/s | 원 추월 .87304093 → 중심 복귀 .35m/s, 조향 −.06257376rad, rejoin |
| 527 / 11.02s | 0.8571505882m/s | 같은 guide/headway 유지, .35m/s, 조향 −.14323125rad |
| 530 / 11.12s | 0.8571505882m/s | 원 운동을 유지한 후속 장면에서 unobserved10으로 거절 |
| 533 / 11.22s | 0.3061252101m/s | masked tail 10.985mm, 기존 기준으로 거절 |
| 536 / 11.32s | 0.3061252101m/s | masked tail 12.037mm, 기존 기준으로 거절 |

늦은 seq560/563에서 새 복귀를 강제로 수락하지 않는다. 특히 seq563은 원 0을 그대로 보존한다. 첫 수락에서 motion-region 거리 약 0.502m를 검사했으며 .918m/s를 목표 .35m/s로 바꾸어 운동 검사를 축소하지 않았다.

최종 생산 소스에 대해 **19개 고유 회귀 항목이 통과**했다. 현재 점유를 ray cast한 전방/대각선/근접 측방 반례, 경로 밖 측후방 반환의 불필요한 거절 방지, 대상 미관측/옆에 있음, 반환 중 앞차 재접근, 현재 전방 경로 소실, epoch reset, 늦은 정지 보존, 기존 비교 기본값 출력 보존, 다음 실제 입력의 경로 유지, 새 proposal 소유 경로, 센터 도달 시 준비된 FOLLOW 인계와 그 STOP 존중을 확인했다. 합성 물체는 현재 반환의 기하 반례이며 움직이는 차량의 미래 안전 증명이 아니다.

실행 관계는 다음과 같다.

- `pytest_v1.log`: 기존 테스트의 ROS fixture import 때문에 수집 실패. 생산 코드 실패가 아니며 원 로그 보존. ROS stub을 추가하지 않았다.
- `pytest_v2.log/xml`: **현재 최종 생산 소스 그대로** 17통과/1실패. 실패 fixture는 옆 물체를 원 벽 뒤에 놓아 ray cast된 반환이 0개였던 구성 오류다. 생산 판정 변경 없이 물체를 보이는 경로 밖 측후방 위치로 옮겼다.
- `pytest_v3.log/xml`: 수정 fixture와 새 proposal/path 검사 2개만 실행해 통과했다. v2의 나머지 17개 테스트 본문과 생산 소스는 이후 변경하지 않았다. 전체 묶음을 불필요하게 다시 돌리지 않았다.
- NumPy 2.5의 기본 2D cross 호환은 진단 runner에서만 기존 determinant 식으로 처리하며 다른 호출은 원 함수를 사용한다. 생산 코드의 플랫폼 이식이나 센서 변경이 아니다. 이번 최종 19개 실행에서 호환 호출 횟수는 receipt에 기록했다.

## 인계

- 이관 검토 대상 생산 파일은 `stage/src/arena_autonomy/arena_autonomy/moving_pass.py`, `bypass_entry_trial.py` **2개**다. stage `__init__.py`는 원본 그대로의 import 지원 파일이므로 이관 대상이 아니다.
- [candidate.patch](candidate.patch), [changes.json](changes.json)에 원 SHA/최종 SHA/common 원본 일치와 실행 증거 SHA를 저장했다. 테스트/프로브는 이 artifact 소유이며 공통에 자동 이관하지 않는다.
- 줄바꿈을 제거한 `read_text().splitlines()` 기준 [내용 diff](content.diff)도 별도 보존했다. `changes.json`은 CRLF/LF 개수와 LF 정규화 내용 SHA를 함께 기록한다. 공통 이관 시 원 줄바꿈을 보존하고 원시 SHA와 내용 일치를 구분한다.
- 주 담당의 기존 ROS/native 환경에서는 `tests/test_moving_pass.py`, `tests/test_moving_pass_medium.py`, `tests/test_bypass_entry_trial.py`의 관련 회귀와 최종 국소 후보의 native 결과를 확인하면 된다. faster 프로필을 함께 적용할 경우 `tests/test_moving_pass_faster.py`도 영향 범위다. 기본값 및 센터 인계 보존을 우선하며 새 전수 주행은 이 결과에 포함하지 않는다.
- 1차 stage 작성은 Windows 전경 Python만 사용했고, 후속으로 위 공통/native 회귀만 WSL에서 실행했다. 공통 생산 파일 이관은 주 담당이 수행했으며 이 담당의 공통 파일 직접 변경은 없다. r478 후속 WSL 5행은 주 담당이 실행·회수한 PID808/잔여0 영수증을 그대로 참조한다.

남은 판단은 현재 속도로의 조기 복귀가 실제 적용된 궤적에서 연속 진행을 만드는지다. 저장 두 호출의 수락은 그 효능이나 10Hz 전체 노드 처리율, 실물 안전을 입증하지 않는다. 현재 stage는 주 담당의 native/회귀 검토 뒤 제한 실행 여부를 결정할 수 있는 후보이며, 원 실패나 관측 한계를 성공으로 바꾸지 않았다.
