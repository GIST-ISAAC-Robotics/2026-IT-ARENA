# r417 fixture revision v2

**준비 완료·실행 전. 제품 8파일은 v1과 동일하다.** v1 fixture, manifest, [related.log](../verify_native_v1/related.log), [related.xml](../verify_native_v1/related.xml), [report.json](../verify_native_v1/report.json)은 원 위치·원 바이트로 보존한다. 이번 manifest는 원 267입력과 원 manifest·실패 결과도 함께 고정한다.

## 실패 원인과 수정 범위

v1은 183검사 중 **179통과/4실패/skip0**, 실제 ROS 생성 단계는 실행되지 않았다. `make_controller()`의 schema5 반환과 runtime `Evaluator.follow`는 `BypassHandoffFollow`다. 실제 정책은 `bypass_handoff.py`의 `self.normal = LocalFollow(self.config)`에 있으며 wrapper의 `reset()`도 `self.normal.reset()`을 호출한다. 직접 `follow.partial_policy`는 존재하지 않는다.

- old-header와 독립 상태 검사는 잘못된 직접 속성 접근의 `AttributeError`가 로그/XML에 기록되어 fixture 오류로 확정된다.
- plan/current fresh child fixture도 `evaluator.follow.partial_policy`를 첫 정책 접근으로 사용해 같은 결함을 가진다. 원 로그/XML에는 부모의 `EOFError`만 기록되어 있으므로, 이 결함이 child 종료의 **유일한** 원인이었다고 소급 단정하지 않는다. 수정 뒤 실행으로 나머지 경로를 확인해야 한다.
- 미실행 `test_crop_edge_ros.py`에도 `node.follow.partial_policy` 접근 5곳이 있어 `node.follow.normal.partial_policy`로 교정했다. 독립 guard의 `guard.partial_policy`는 올바른 소유 구조이므로 그대로 유지한다.

새 pure fixture는 `follow`/`one`/`two`/`evaluator.follow`의 실제 `.normal.partial_policy`를 사용한다. 제품에 alias를 추가하지 않고, 검사 기대·선택·허가·reset 의미도 변경하지 않는다. 두 fixture의 module 이름에 `_v2`를 붙여 multiprocessing spawn이 원 v1 test module을 잘못 다시 읽지 않게 한다. runner는 이 두 revision 파일과 revision manifest 경로만 바꾸며 기존 launch AST·8관련회귀·실제 ROS 생성 순서를 유지한다. 변경 원문은 [fixture.patch](fixture.patch)에 있다.

r414 정책 SHA-256은 계속 `576966260591adf169c426f33c921df0586ca75f31e9e973c69be94b2c268425`다. staged 제품 8파일 전체 SHA는 [static_preparation.json](static_preparation.json)에 기록한다. Windows stdlib로 원 입력 불변·AST만 확인하며 WSL/ROS/장치/주행을 실행하지 않는다.

## 실행 담당자 고정 명령

이 작업 환경의 ROS는 **Jazzy**다. 원 v1 README의 Humble 안내는 당시 오류로 보존하고 이 revision에서는 아래 명령을 사용한다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B artifacts/validation/2026-11-13/crop_edge_runtime_r417_v1/revision_fixture_v2/run_verify.py --output artifacts/validation/2026-11-13/crop_edge_runtime_r417_v1/verify_native_v2
```

원 v1 출력은 덮어쓰지 않는다. `verify_native_v2/{related,actual_ros}.{log,xml}`와 `report.json`에 새 결과를 남긴다. 관련 검사 통과 후 실제 두 노드 ON 생성·원시 frame/time/epoch·native child 회수 검사를 진행한다. 실패·skip·입력 해시 변경이면 중단한다. 새로 드러나는 실패는 별도 원인으로 보고하며 기존 179통과나 r414 결과를 새 실행의 성공으로 대체하지 않는다.

ROS domain223/localhost, 센서/serial/모터/Gazebo/ARM 없음, spin/control/worker.offer 미호출, 공통소스 미변경 범위는 v1과 같다. 시험 뒤 모든 자식의 정상 종료와 입력 불변 확인이 필요하다. 운영 채택·실제 guard 주행·완주 판정은 후속 r418의 별도 결과다.
