# r555 OFF 대표 — 출발 전 launch 오류

2026-11-14 GPT 실행·회수. 승인된 OFF 실행을 **한 번 기동**했으나 ROS 노드 초기화가 실패하여 실제 교통 주행은 수행되지 않았다. **주행 성능 실패와 구분하는 출발 전 실행기 오류**다. 원 실패와 snapshot은 보존했고 source 수정·재시작·추가 주행을 하지 않았다.

원 `passed=false`, `run_valid=false`, `harness_valid=false`, `shutdown_clean=false`를 그대로 유지했다. `/trajectory`, scan, autonomy/safety 상태는 모두0행, actuation trace0개, ego pose0행이며 GREEN/case window가 없다. 양수율·정지시간·충돌 회피·상대 귀속 반환수는 평가 불가다. actor 평가 pose가 존재하는 것을 자차 주행 근거로 바꾸지 않는다.

## 오류와 회수

[simulation.log](run1/simulation.log)의 `vehicle_marker_profile_path:=` 빈 파라미터가 ROS CLI에 전역 전달되어 `Failed to parse global arguments`와 `Couldn't parse parameter override rule`가 발생했다. controller·bridge·vehicle create 등13개 child가 초기화 실패했다. [marker_course.launch.py](marker_course.launch.py)의 OFF 빈 문자열 전달이 해당 실행 입력이다.

PID **47746**의 cmdline/cwd를 확인한 후 SIGINT를 한 번 보내 기존 validator의 finally 회수를 요청했다([영수증](execution_v1/interrupt_failed_start.json)). 원 report의 최종 error는 `RuntimeError: interrupted`이며, 최초 파라미터 오류는 원 simulation 로그에 남아 있다. STOP 서비스 성공과 정상 child 전체 종료는 증명하지 못했다. STOP 관련 false 판정을 덮어쓰지 않았다.

Gazebo server_control 종료 응답은 성공, 강제 종료 없음, 미회수 child/test/owned PID0이다. launch 자체 exit0과13개 child 실패를 구분한다. [독립 사후 PID 검사](execution_v1/owned_processes_after.json) 후 WSL을 주 담당에게 반환했다. 그 뒤 Windows 보고서만 작성했다. Jetson 접근0.

## 입력과 수집

r553 대비 공통 제어 변경은 승인된 `bypass_entry_trial.py` 한 파일(SHA354239c472dc95a116259ec3051239bdb945feddea30f8c51f3a08b788971185)뿐이다. r554 관련217통과/기존 fixture2실패 근거를 그대로 보존했고 전체219통과로 기록하지 않았다. 실제 import12개, 경쟁0, native 바이너리 불변, 기존 symlink build 재사용을 확인했다.

희소 상부 actor·마커 OFF·원12sim초·RTF.25·8sim초 무진행·원STOP/판정은 준비대로였다. actual launch/SDF의 **7조향 검사 통과**는 ROS 초기화 성공과 별개다. marker ON은 실행하지 않았다.

필수 입력468개/ZIP 불변, 실제 startup source map431개와 snapshot의 누락·SHA불일치0을 확인했다. 실행 담당에게 위임된 prepared/ZIP 대조는 [root_execution_review.json](root_execution_review.json)에 기록했다.

- [prepared.json](prepared.json) SHA `48396e1e322199b367a4eced9b8c40772c263d4a5fbcd703e1c2cf42a7c3729b`.
- [sources.zip](sources.zip) SHA `5b9b2a583fb885e7ce2f3b8b9ddc5815c757c4da3d81961797f46afef8e49d6d`.
- [원 report](run1/report.json) SHA `6a7ce91f731b97ce97da3f3a1aacc383b25b8f24845265d6324719a8381dfd80`.
- [실제 startup 입력](actual_execution_inputs.json), [기술 실패 영수증](startup_failure.json), [원 수집](evidence_collection.json).

[수집 종료코드](execution_v1/collection_exit.json)는 process0/integrity0/summary1/assessment1/generic collection0이다. summary는 미생성 `case_window` 접근, assessment는 옛 기본 vehicle.yaml 고정 SHA 기대 때문에 실패했다. 이 후처리 오류도 [별도 기록](collection_errors.json)으로 보존했다. follow_decisions와 stop_trace 미생성을 누락 증거로 남겼으며 임의 생성하지 않았다. 원 r553 단독 정체 결과는 별도 원본으로 유지한다.
