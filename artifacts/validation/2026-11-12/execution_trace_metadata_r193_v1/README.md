# 실행 기록 메타데이터 호환과 PC·Jetson 저장 재생

r192의 `unknown trace event`는 제어 결과 오류가 아니라 기존 계측기가 실행 설정 이벤트를 몰랐던 문제였다. `decision_submission_policy`, `current_path_recheck_policy`, `delivery_delay_profile`을 handoff 기록의 감사 정보로 보존하도록 수정했다. 이 정보가 제어 상태/시각을 바꾸거나 기록된 지연을 다시 실행하지 않는다. 순번·완료 수·출력 대조와 그 밖의 미등록 이벤트 거절은 유지한다.

관련 검사 최초56통과/2실패는 패키지가 모두 단일 `.py` 파일이라는 오래된 테스트 가정 때문이었다. 실제 의존 파일·하위 패키지 `__init__.py`/모듈까지 검사하도록 고쳤다. 이후9검사 통과로 중복을 제외한 관련58개가 통과했다. `retest.log`의 ROS 환경 미적용 수집 실패도 보존하고, 환경을 적용한 `retest_ros_env.log`를 결과로 사용했다. 최신 변경 계약과 전체 등록 소스 연결을 확인했다(`provenance.log`).

## 제한 계측 결과

명시 `selected_r181` 헤더의 r192 파생 입력을 그대로 사용했다. 실제 입력·출력439개와 본문 해시는 원 r182 기록 그대로다.

- **PC WSL:**439판단 모두 일치. 1회 warmup 뒤1회 계산의 전체 중앙32.417ms·p95 51.053ms. 원 입력/필수 소스 불변이다.
- **Jetson:** 새 폴더에116파일을 전송하고 archive/모든 파일 해시를 검증했다. Python3.10.12/NumPy1.21.5/aarch64에서 출력 대조가 실패해 **속도 계측에 진입하지 않았다**. 원 입력/소스는 불변이다. 수치 환경별 차이는 후속 r194에서 분리한다.

두 경우 계산 비용의 범위는 이력 복원·스캔 운동 투영·계획기이며 ROS 수신·현재 경로 재검사·독립 보호·영상·구동기·실제 취득시각은 포함하지 않는다. PC 결과를 Jetson 성능이나 전체 실시간 성립으로 해석하지 않는다. GPU·장치 포트·모터 명령·전원 설정은 사용/변경하지 않았다.

`pc/report.json`, `bundle_manifest.json`, `bundle_audit.json`, `remote_run.py`, `remote_session.log`, `jetson_result/`가 근거다. 원 실패 r192를 덮어쓰지 않았다. 원격 `/home/q/it_arena_follow_profile_r193_20261112`는 이번 저장 계측용 격리 폴더이며 운영 설치를 바꾸지 않았다.
