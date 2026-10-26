# M4 두 번째 출발 검증

[사전 등록](PLAN.md)의 별도 정방향 시험이다. 기존 run64의 새 신호 없는 정지 성공과 기존 교통 실패는 보존한다. 현재 구현은 시험기에서만 명시 선택하며 제어기·독립 보호·센서/물성·기본 운영 모드를 바꾸지 않는다.

## 구현 범위

- `scripts/m4_second_start.py`: 재ARM 대기 후 실제 렌즈의 빨강/초록을 각각 한 번 요청한다. 서비스 응답·렌즈 적용·새 세대 픽셀 출발 사실을 구분한다. 응답 유실·거절·세대/boot/권한 변경은 재시도 없이 실패한다.
- `scripts/validate_g1_traffic.py`: `new_race_restart` 명시 선택일 때만 위 어댑터와 별도 평가기를 생성한다. 기존 운영 시험과 기본값은 그대로다.
- 평가기는 새 신호 이전의 정지와 이후 실제 1 m 진행·재출발을 분리한다. 원래 교통 판정은 보존한다. 공식 4-red·그리드·실물 운영 완료는 아니다.
- [수정 전 시험기 소스](before_harness_sources.zip)와 후속 출처 계약을 보존한다. 앞선 M3의 원문 순환/복귀 관문 이관은 별도 계약이며 M4 제어 기능으로 채택하지 않는다.

## 실행

| 묶음 | 결과 |
|---|---|
| run1_pure_regression | 05:20:07–05:20:10 KST, 51통과, exit 0. 소스·검사 전후 동일. 이벤트 시각 교차 검사를 추가하기 전 판본이며 원본 로그 보존. |
| run2_integration_preflight | 05:24:24–05:24:28 KST, 수집 실패(exit 2). 명시 PYTHONPATH가 ROS 환경 경로를 덮어 `xacro` import 실패 3건. 테스트·빌드·설치 검사는 미수행, 전후 해시 동일. 잘못된 launch 경로를 지정한 보조 사전 검사 실패도 원문 보존. |
| run3_environment_preflight | 05:26:14–05:26:41 KST, 202통과·1실패(exit 1). ROS import 정상·소스 불변. 초기 추가 의존성의 후속 변경 사슬을 추적하지 않던 옛 코너 출처 검사가 `observation_support.py`의 새 명시 변경을 거절했다. 빌드/설치 검사 미수행. |
| run4_source_chain_build | 관련 출처/코너 29통과(exit 0), `arena_autonomy --symlink-install` 빌드 성공. 소스 경로 오버레이 없는 새 bash에서 5모듈과 launch 설치본 SHA 일치·신규 helper 발견·전후 소스 불변. 옛 계약 해시는 변경하지 않았다. |
| [run5_second_visual_start](run5_second_visual_start/run/report.json) | 05:35:51–05:39:48 KST, Gazebo 1회 exit 0. 별도 M4 11항목 통과, 원래 교통 `passed=false` 보존. 신호 전 3.683 s 정지, 두 번째 초록 적용 후 0.585 s 실제 출발·3.146 m 진행. 재생 418/418·노면 9,692표본 이탈 0·마지막 STOP/정상 종료/잔존 0·등록 83소스 불변. |

run5의 첫 보조 사전 검사는 잘못된 manifest 파일명 때문에 주행 전에 실패했다. 경로를 바로잡은 사전 검사 뒤 **Gazebo는 한 번만 실행**했으며 앞선 로그도 보존했다. 원래 교통 HOLD 미달과 의도된 중간 STOP 판정은 바꾸지 않는다. 등록된 운영 STOP만 별도 평가에서 구분했고 마지막 밀집 STOP은 통과했다. 실제 임시 신호등 RGB를 사용했으며 공식 4-red/그리드·실물 결과가 아니다.
