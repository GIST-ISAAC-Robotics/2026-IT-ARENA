# r558 단독 코너 확인 레시피

**Windows 준비만 완료하는 폴더다. root GO 전이며 빌드·소스 동결·WSL/Gazebo·실제 주행은 0회다.** r560 교통 12 sim초 실행과 STOP/PID0 회수 뒤 WSL 소유권을 인계받고 root가 승인한 경우에만 다음 단독 1회를 수행한다. 실패 자동 반복은 없다.

[r553 원본](../measured_steering_radial_lap_r553_v1/RESULT.md)의 actor0, grid3, 시설3개, 동일 fieldworld·실측 조향 YAML, `g1_open_1p5`/`medium_v1`, RTF 설정0.25, 최대80 sim초와 8 sim초 무진행 STOP을 유지한다. 원 진행·노면·충돌·STOP·정상 종료·입력 무결성 판정도 유지한다. 첫 직선 일부 벽의 굴곡/틈새만 있는 기존 코스이며 전 구간 변형 코스는 아니다. 중앙 조향 상한은 `0.28475436568260193 rad`; 바퀴각 실측이 아닌 서보 count의 CAD 파생값이다.

r553 대비 현재 공통 소스 차이는 r557의 `current_wall_reacquire.py`, `current_path_recheck/_geometry.py`, `race_runtime_policy/retained_recheck/_geometry.py`와 r554의 `bypass_entry_trial.py` 네 파일이다. 저속 재획득은 실제 반환점+고정55mm 여유를 사용하며 미관측 상대 이동 상계 보수성을 줄이는 정책이다. 실제 점유·STOP·고속 경로는 유지하지만 실차/동적 상대의 모든 움직임에 대한 안전 증명은 아니다.

- [준비 확인](preparation_check.json): 원 config/명령의 경로 치환 외 동일성, world/시설/기대 헤더/판정 파일 동일성, 원 실행/정체/ZIP 보존, Python 구문 검사만 기록한다.
- [출처와 원본 SHA](recipe_provenance.json), [소스 선택 제안](source_selection.json): 준비 시점의 네 파일 차이를 기록했다. **모든 실행/빌드/동결 권한은 false**이며 실행 당시 최종 선택·실제 import·무결성 검증을 대신하지 않는다.
- [실행 순서와 명령](commands.json): root가 r560 회수·독점 소유권·최종 SHA를 확인한 뒤 승인한다. `build_once.sh` → `prepare_once.sh` → 새 동결 SHA에 결합된 `root_execution_review.json` → `execution_v1/execute_once.sh` 순서다. `execution_v1/observe.py`와 `stop_once.sh`의 원 8 sim초 정체 STOP을 유지하고 `finish_collection.bash`로 회수한다.

r553의 `sources.zip`, `run1`, 성공/실행 승인 영수증을 이 폴더로 복제하지 않는다. 준비용 경로 변경 외 연결 변경은 `prepare.py`의 기준 폴더를 r553으로 지정하고 `preflight.py`에 이번 변경 모듈 `bypass_entry_trial`의 실제 import 검사를 더한 것뿐이다. 실행 담당이 이 폴더를 넘겨받기 전까지 공통 소스·설정·장치에는 변경을 가하지 않았다.
