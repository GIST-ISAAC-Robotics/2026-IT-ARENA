# r551 미표시 미리보기 부하 공통 이관

2026-11-14. r550 종료·입력 동결 해제와 주 담당의 이관 승인을 받은 뒤, 검토 당시 원본 SHA가 현재 공통과 일치함을 확인하고 적용했다. 장치/서버 실행 없이 Windows에서 검사했다.

## 반영 내용과 범위

- `race_ops_page.py`: 숨김 탭의 `/api/state` 조회에는 `preview=0`을 붙인다. 카메라 선택 해제 또는 숨김 상태에는 JPEG를 새로 요청하지 않으며, 진행 중 요청이 완료되더라도 숨김 상태에는 표시하지 않는다.
- `race_ops_node.py`: `preview=0`은 해당 요청의 미리보기 lease 갱신과 `lidar`/`camera` payload만 생략한다. 운영 `client_seen` heartbeat는 유지하고 공유 cache·전역 enable·다른 보이는 탭의 lease를 끊지 않는다. query가 없으면 기존 전체 응답과 lease 갱신을 유지한다.
- `tests/operator_panel_page_checks.js`, `tests/test_operator_panel_page.py`: 기존 r535를 포함한 11개 검사를 그대로 두고 새 DOM 대표 3개를 추가했다.
- 신규 `tests/test_operator_preview_idle.py`: 실제 공통 Handler AST와 기존 fake ROS fixture로 owner 만료·다른 owner 보존·기본 전체 응답을 검사한다. stage/baseline artifact 의존은 없다.

공통 production은 승인된 stage와 텍스트가 같고 기존 LF를 유지한다. node의 `Handler.do_GET` 외 AST는 원본과 동일하다. 제어·START·manual heartbeat·STOP 관련 코드는 바꾸지 않았다.

## 결과

**Windows 17/17 통과, 1.75초**, 오류·실패·skip0이다. 공통 페이지 DOM14개와 공통 route3개이며 기존 r535 재연결3개를 포함한다. 원본 새대표 실패와 stage 첫 fixture 오판/수정 결과는 [stage 결과](../stage/RESULT.md)에 보존했다. 이번 이관에서는 실패 없이 한 번 실행했다.

```powershell
$env:PYTHONPATH='src/arena_autonomy;src/arena_vehicle_interface;tests;scripts;artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'
$env:PYTHONIOENCODING='utf-8'
python -m pytest tests/test_operator_panel_page.py tests/test_operator_preview_idle.py -q --tb=short -p no:cacheprovider --junitxml=artifacts/validation/2026-11-14/panel_preview_idle_r551_v1/common_transfer/pytest.xml
```

변경 전후 SHA·파일 목록은 `manifest.json`, diff는 `r551_common.patch`, 실행 결과는 `pytest.log`·`pytest.xml`, 보존/범위 확인은 `verification.json`에 있다. `*.before`와 `*.after`는 이번 이관 기준이며 기존 다른 작업 변경을 보존한다.

## 남는 경계

숨김 owner의 기존3초 lease가 만료되면 기존 정책대로 구독·worker가 종료된다. 다시 표시한 후 미리보기를 원하면 기존 **미리보기 적용**을 누른다. 자동 재활성화는 추가하지 않았다. 진행 중 전송을 취소하지는 않으며, 문서는 보이지만 viewport 밖인 경우는 이번 범위에 포함하지 않는다. 실제 브라우저 백그라운드 timer와 Jetson CPU/네트워크 절감량은 미측정이다.

WSL/실제 ROS/Gazebo/Jetson/물리 장치/새 서버 접근0, 설치·커밋·푸시0이다. 위5개 파일의 소유권을 주 담당에게 반환한다.
