# r457 최신 차체 actor0 단독 1랩 준비

**adapter 비주행 준비를 마쳤으며 주 담당이 이관한 r459 정책으로 최종 준비한다. 실제 주행 완료/통과가 아니다.** 초기 adapter 검사는 r452 소스에서 수행했다. 이후 r459 native 비교·정책 검토/이관을 마친 주 담당의 지시로 r452 대비 소스5파일+시험1파일만 허용한다. [정책 변경목록](../bounded_guard_approach_r459_v1/changes.json)과 [통합 영수증](../bounded_guard_approach_r459_v1/integration.json)의 before/after SHA를 모두 검사하며 다른 추가/변경/누락은 거절한다. arena_autonomy1회 build는 exit0이며, 뒤의 헤더 바이트 검사에서 artifact의 CRLF→LF 복사 차이로 중단됐다. JSON 내용은 exact였고 원 실패/사본을 보존한 뒤 r452 원바이트로 복원했다. `build_v2/session_resume.sh`는 빌드 없이 실제 import/native→고정CLI의 ROS init 직전 차단→새 archive 동결→종료 감사를 같은 세션에서 수행한다. 완료 여부와 SHA는 [준비 영수증](build_v2/freeze.json)에 기록한다. 주 담당 `root_execution_review.json` 뒤 고정 `execution_v1/execute_once.sh`로1회만 실행한다. 구정책 단독 추가랩은 실행하지 않는다.

## 비교 조건

- r452는 `eval_g1b_actor`, `eval_second_lead` 차량2대가 있었다.20.18m 정체/미완주는 교통 결과이며 완전 단독 실패로 분류하지 않는다.
- r457은 위 차량 모델/pose bridge/set_pose/actor 수신 대기/상호 간격·뒤따름·actor 접촉 검사를 제외한다. old610 신호·ID20·ID30 지주3개와 Contact world plugin은 XML element exact로 유지한다. 차량을 원거리로 숨기거나 빈 입력을 자유공간으로 채우지 않는다.
- `vehicle.final_cad_2026_11_13.yaml`: 물리 차체199.7×150mm, planner/evaluator 보수 외곽207×150mm, 축간128mm, 윤거123.9mm, 타이어지름65mm. 실장 측정 완료 주장이 아니다.
- `b0390_c1_cad_20261113_final_frontwire_6mm.yaml`, C1 native10Hz/sequential/both/4개 자기마스크, RGB/IMU/엔코더, r452 header/선택 플래그 유지. `g1_open_1p5`, control50Hz, RTF0.25, step1ms, road_level, marker blank20/30, red1초, slot3 유지. 구현은 승인된 r4595파일을 포함하며 r455는 미통합이다.
- slot3 x9.89m/y3.52m/yaw1.5708rad. 평가 중심선44.64m를 연속 누적해1랩으로 판정하며1m 초과 투영 점프는 오류다. 원 교통1랩 기준이며 이전 별도 solo 도구의 +2m 여유와 구분한다.

## adapter와 판정

기존 `run_new_course_solo`는 JR60P20·다른 속도프로필이며 하위 평가기는20×15cm 고정이라 그대로 쓰지 않았다. r452 동결 평가기의 artifact 사본에서 actor 실행만 제거한다([diff](solo_runner.patch)). 공통 제어/보호 코드 변경·monkeypatch는 없다.

`run.py`는 r452 시설 기하·launch wrapper를 재사용한다. `solo_evaluation.py`는 기존 수치/coverage/연속 진행/바퀴 노면 helper를 쓰고 원 finalize의 actor 전용2조건만 무차량 XML/pose publisher 부재 검사로 바꾼다([diff](solo_finalize.patch)). `/drive`→독립 보호→`/drive/safe`, sensor-only graph, 실제 차체 STOP·MCU STOP·기한진단·종료·소스 무결성 검사를 유지한다.

제거한 차량의 `/eval/g1b_actor/contact`는 N/A이며 침묵을 무접촉 증거로 세지 않는다. 벽/old610 평면 겹침·STOP 벽 충돌을 유지하고 밀집 자세의 같은 외곽 겹침을 기록한다. 실제 접촉력/연속 물리 무접촉 증거가 아니다. 차체 road 이탈은 원 full-course처럼 별도지표이며3바퀴 이상 중심 이탈0이 노면 판정이다.

## 실행·회수

준비 후 고정 명령(아직 실행 승인 전):

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_full_lap_r457_v1/execution_v1/execute_once.sh'
```

80sim초 상한·최초8sim초 무진행 ownedSTOP·원 벽 겹침/경로이탈 중단 기준 유지.1랩을 채우면 원 평가기 STOP/회수. 시작150wall초·실행1500wall초·STOP120wall초. 원 교통 판정의 이동 후 .05m/s 미만 연속2.5sim초 상한도 유지하므로, 랩 완료 여부와 정지 최소화/전체 통과 여부를 따로 기록한다. `observe.py`와 `stop_once.sh`는 자신의 runner만 대상으로 하며 성공할 때까지 반복하지 않는다.

원 report/trace/native scan/완전 plan 제출·완료·header/MCU/STOP/프로세스/partition/입력SHA를 회수한다. 첫 정체의0명령 이유·유한 반환/자기마스크/미관측·parent발행·보호상태·command_expired·sim/wall 계산비용을 연결한다. actor0 정체를 교통 차량 때문이라고 설명하지 않는다. 기준랩 뒤 출발/벽 형상 변형은 주 담당 r460의 별도 준비 범위다.

첫 WSL 읽기 ps가 `Wsl/Service/E_UNEXPECTED`로 실패했다. 새 build/ROS/Gazebo는 시작되지 않았고 주 담당이 Ubuntu-24.04를 복구한 뒤 비주행 준비를 재개한다. 원 오류는 [receipt](preflight_v1/wsl_unexpected.json), 복구는 [root receipt](../solo_complete_r457_wsl_recovery.json)에 보존한다.

## 완료한 준비 증거

- [Windows 정적](preflight_v1/windows_static.json): r452 common1307개 SHA 동일, 차량2개만 제거·시설3개/plugin exact, launch 값·timeouts 유지.
- [설치 재사용](build_reuse_v1/receipt.json): 기존3패키지 import 위치와 source SHA, native SHA 일치. 새 build0.
- [비주행 entry/helper](preflight_v1/result.json): 실제 고정 CLI 처리 후 `rclpy.init` 직전에 예외로 차단. ROS init/Gazebo/ARM0. 첫 합성 fixture가 중심선4꼭짓점만 사용해1m 투영 점프를 일으킨 실패는 [원 log](preflight_v1/output.log)에 보존했다. 촘촘한 중심선으로 fixture만 고친 뒤 남은 [helper7검사](preflight_v2/helper_result.json) 통과.
- [최종 계약](preflight_v2/windows_final_contract.json): 원 finalize 대비 actor 전용2키만 변경 exact. 정상1개와 STOP·벽·sensor-only·source·shutdown·예상 밖 actor 반례7개 확인. 검토 중 추가했던 밀집 벽 acceptance gate는 제거하여 원 r452의 벽/STOP 판정선을 유지했다. 밀집 겹침 계산은 진단 지표로 남기며 이전7검사의 원 결과는 소급 수정하지 않았다.
- [초기 회수](preflight_v2/cleanup.json): 당시 소유 Python PID0, build/ROS init/Gazebo0으로 WSL을 반환했다. 이후 r459 통합 뒤 주 담당이 WSL을 다시 배정했다. 최종 준비의 build/import/고정CLI는 [receipt](build_v1/receipt.json), prepared/archive SHA는 [freeze](build_v2/freeze.json), 종료는 [session cleanup](build_v2/session_cleanup.json)에 별도 기록한다. 헤더 줄바꿈 전송 이슈는 [원 실패](build_v1/import_cli.log), [원 종료](build_v1/session_cleanup.json), [바이트 복원](build_v2/header_fix.json)에 보존한다. 초기 결과를 최신 정책 주행 성공으로 소급하지 않는다.

기준랩 뒤 출발·벽 변형은 root의 r460 담당이다. simulation.launch.py의 `test_spawn_*`는 `world_override`에서만 적용되지만 같은 옵션이 road_level/blank20_30/evaluation_actor_sdf와 충돌한다는 접점을 전달했다. r457에서 그 범위를 구현하지 않는다.
