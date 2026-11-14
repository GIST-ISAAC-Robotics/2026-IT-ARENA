# r551 미리보기 비표시 낭비의 최소 stage

**공통 미반영·root 검토 대기입니다.** OFF/조회 중단 처리는 이미 있으므로 그대로 유지했습니다. 실제 남은 낭비는 숨긴 탭의 미리보기 lease 갱신과, 로컬 카메라 선택이 꺼진 화면의 JPEG 수신 후 폐기였습니다. page와 HTTP GET의 작은 복사본 수정만 준비했습니다.

## 이미 제어된 부분

- `race_ops_preview.py:83`: 기본 OFF, worker/미리보기 구독 없음.
- `:80/101/112/141`: 마지막 설정 client만 lease를 갱신하며3초 만료 또는 명시 OFF 뒤 기존20 ms node tick에서 구독·cache·pending을 해제하고 worker가 종료합니다.
- `:81/106/144`: LiDAR5 Hz·camera2 Hz, 종류별 최신 pending1개. 오래된 세대의 변환 결과는 OFF 뒤 버립니다.
- `:170/181`: 입력1초 이상 오래됨을 표시하며 오래된 JPEG는 제공하지 않습니다.
- 기존 `test_operator_preview.py`는 OFF/lease 해제·단일 pending·이전 세대 폐기·다른 client가 lease를 갱신하지 않는 계약을 이미 검사합니다. 해당 기능을 새로 만들지 않았습니다.

## 확인된 두 낭비와 변경

현행 page는 document visibility를 보지 않고200 ms마다 `/api/state?client=...`를 조회합니다. 현행 Handler는 이를 매번 preview keepalive로 사용하므로 상태 조회가 계속되는 숨긴 탭도 ON 구독/변환을 유지할 수 있습니다. 또 `cameraPreview()`는 checkbox OFF를 JPEG 수신 후 확인하므로, 다른 탭이 켠 camera 상태를 조회하는 미선택 탭은 파일을 받고 버릴 수 있습니다.

stage는 숨긴 document의 상태 조회에만 **`preview=0`**을 추가합니다. Handler는 그 요청의 preview keepalive만 생략하고 응답의 `lidar`/`camera`를 null로 만듭니다. 전역 desired/enable·shared cache·구독·소유 client를 직접 변경하지 않습니다. 운영 `client_seen`·START/manual heartbeat와 상태 조회 주기는 원래 흐름 그대로입니다. query를 생략한 기존 API 소비자는 기존 응답과 lease 갱신을 그대로 받습니다.

page는 숨김 또는 로컬 camera 미선택이면 JPEG 요청을 시작하지 않습니다. 이미 진행하던 요청도 숨김 후 응답이 오면 새 Blob URL/화면을 만들지 않습니다. 숨긴 document는 미리보기 canvas/render 작업도 생략합니다. 최초 요청 하나가 이미 전송된 경우 그 전송을 취소하는 기능까지 추가하지 않았습니다.

숨긴 탭이 현재 preview owner이면 마지막 유효 조회부터 기존3초 lease를 통해 구독/worker가 정리됩니다. 다른 보이는 탭이 현재 owner이면 그 탭의 정상 조회가 계속 갱신하며 숨긴 비소유 탭은 이를 끊지 않습니다. lease가 실제 만료되어 OFF가 된 뒤 다시 보고 싶으면 돌아온 화면에서 기존 **미리보기 적용**을 누릅니다. 화면 아래로 스크롤해 미리보기가 viewport 밖에 있는 경우의 자동 ON/OFF는 이번 범위에 추가하지 않았습니다.

## Windows 근거

| 검사 | 결과 |
|---|---|
| 원본의 숨김 조회·미선택 JPEG·숨김 뒤 JPEG 완료3대표 | 0/3 통과, 실패 원문 보존 |
| stage DOM | 기존11개(r535 재연결3개 포함)+새3개 = **14/14 통과** |
| 실제 Handler GET + 기존 fake subscription/worker | **3개 통과,0.28초** |
| 숨김 owner | core client heartbeat 유지, preview lease 갱신 없음, 기존 만료 뒤 구독0/cache0/worker 종료 |
| 숨김 비소유 탭 | 현재 보이는 owner의 lease/구독2개/shared cache·전역 ON 유지 |
| query 없는 GET | 원본 Handler와 응답 전체 및 lease 동작 동일 |

첫 DOM 실행의 새 JPEG 완료 검사1개는 기존 DOM fixture가 HTML의 `hidden` 속성을 false로 초기화한 탓에 잘못 실패했습니다. 실제 관심 동작인 새 Blob URL 생성0과 sequence 미반영으로 검사만 바로잡았으며 page stage 소스는 바뀌지 않았습니다. 첫 결과는 `dom_results_first.json`, 최종은 `dom_results.json`에 있습니다. 라우트 결과는 `pytest.xml`입니다.

원본/후보 소스·SHA는 `*.before`, `manifest.json`, 최소 변경은 `preview_idle.patch`, 실행 코드는 `prepare.py`, `run_dom_checks.py`, `test_routes.py`입니다. 새 서버나 포트를 만들지 않았으며 ROS는 실제 실행하지 않고 기존 구독 대역만 사용했습니다. 실제 browser visibility 이벤트/백그라운드 timer 정책·Jetson CPU/네트워크 감소량은 미측정입니다.

공통·r550 동결 입력·기존 검사 파일 변경0, WSL/ROS/Gazebo/Jetson/물리 장치 접근0입니다. root 검토 전 이 stage를 활성화하거나 이관하지 않습니다.
