# r466 일시중지 인계

**보존된 중지 당시 기록입니다.** 사용자 재개 뒤 [v2 수정·Windows/ROS 결과](revision_v2/README.md)를 완료했습니다. 아래의 ‘수정 미적용’은 중지 당시 사실이며 현재 결과는 [RESULT.md](RESULT.md)를 따릅니다.

사용자의 일시종료 요청으로 **추가 ROS 재시험·수정 묶음을 시작하지 않고 중지**했다. 공통 미통합이다. root가 WSL을 회수했다.

## 확정 결과

- 구현 stage의 Windows 197개 통과는 당시 결과로 유지한다. 기본/OFF·packet 계약·계획/현재/독립 guard 본문 I/O-double 검사다.
- 이후 실제 ROS 대표를 한 번 실행했다. Windows에서 제외한 ROS import 2개는 실제 ROS 환경에서 통과했다.
- 실제 VisionProcess→MarkerChannel publish→ROS 수신까지 packet 20개가 기록됐다. 그러나 정면 ID1이 `ambiguous_duplicate`로 제외되어 `regions=[]`였고, **정면 감속은 실패**했다. 원 `execution_v1/run1`과 stdout/stderr/completion을 보존한다.
- 정식 판정은 부재 `.75 m/s` 유지 통과, front cap 실패에서 중단했다. side/TTL/마지막0까지 입력·원 로그는 이미 실행돼 남았지만, 실패 뒤 사례를 대표 통과로 올리지 않는다. 원 `/drive/safe` 마지막0 표본은 존재한다.
- 영상 child는 `forced=false, exitcode=0`으로 join했고 3개 ROS node destroy를 기록했다. 최종 `pause_pid_audit.json`에서 runner 779/child 795 모두 absent, r466 명령줄 매칭 0개다. 이전 진단의 단순 PID존재 목록은 프로세스 동일성 판정 근거가 부족하며 최종 감사와 구분한다.
- 실행 snapshot 232파일은 회수 시 불변이었다. 구현 stage ZIP SHA는 기존 `23ac47b62f40d53a755acf40748d09f0482c74eb8a4670a8feba3ea410004363` 그대로다.

## 중복 quad 원인과 미구현

`duplicate_diagnostic.json`은 원 대표와 같은 합성 RGB 두 장을 OpenCV 4.6으로 읽은 좌표다. 새로운 ROS 시험은 아니다.

정면 이미지 1280×720에서 하나의 ID1에 다음 두 quad가 나왔다.

- outer: `(600,400),(679,400),(679,479),(600,479)`; 평균 변 79 px.
- inner: `(605,406),(673,405),(674,473),(606,474)`; 평균 변 약 68.007 px.
- 중심은 모두 `(639.5,439.5)`, 중심차 0. 면적은 6241/4625 px², inner가 outer 안에 있어 IoU=4625/6241≈0.7411이다. 변 크기비 약1.1616, 대응 corner 최대차 √61≈7.810 px이다.
- 현재 geometry의 고정 2 px 동일성 조건이 이 **동일 합성 코드의 중첩 검출**을 모호한 중복으로 제외했다. 측방 48 px 영상은 ID1 하나만 나왔다.

root는 거의 겹친 동일 ID를 중심/크기비/IoU로 묶고 대표 하나를 고르는 최소수정을 허용했다. 가장 넓은 검은 quad를 고르면 안쪽 검출 때문에 추정 거리가 약16% 늘어나는 문제를 피할 수 있다는 설계 후보가 있다. 다만 이는 단일 합성 입력의 추론이며 실물 코드 외곽·절대 정확도를 증명하지 않는다. 분리된 동일ID, 크기 크게 다른 quad, decoded corner 방향 불일치 반례를 함께 유지해야 한다.

**수정은 아직 적용하지 않았다.** 코드 적용 도구 호출이 중지 메시지로 dispatch 전에 취소됐다. 기존 2 px 조건 그대로이며 새 중복 알고리즘·추가 검사·두 번째 ROS 대표는 없다. 구현하지 않은 수정을 완료로 기록하지 않는다.

## 명시 재개 후 다음 한 가지

1. `vehicle_marker_geometry.py` stage에서 중첩 quad 대표 선택만 수정하고 위 원 좌표+필수 반례의 Windows 검사를 추가한다. raw ray/벽/free/현재 주행 정책은 건드리지 않는다.
2. 원 `execution_v1`과 `/home/jinhyeong/it-arena-r466-ros-a`를 보존한다. 새 `execution_v2`/새 workspace로 snapshot·한 번 게이트를 준비해야 한다. 현재 v1 `run_once.sh`를 재호출하지 않는다.
3. root가 WSL을 다시 배정한 뒤 같은 소규모 ROS 대표만 확인한다. 새 주행·포트·Motor bridge는 필요 없다.

현재 재개용 Windows 검사 명령의 기존 진입점은 `py -X utf8 artifacts/validation/2026-11-13/vehicle_marker_r466_v1/run_windows_tests.py`다. 수정 후 필요한 subset으로 새 로그를 남기고, 전체 회귀를 기계적으로 반복할 필요는 없다. 새 v2 Linux 실행 명령은 아직 만들지 않았으므로 실행 가능한 것처럼 제시하지 않는다.

현재 본 담당의 실행 프로세스·점유 WSL 자원은 없다. root 승인 없이 공통 통합하지 않는다.
