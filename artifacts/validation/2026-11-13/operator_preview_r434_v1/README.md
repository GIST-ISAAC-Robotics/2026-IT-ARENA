# r434 선택형 센서 미리보기 stage

r429 통합본을 기준으로 `race_ops_node.py`, `race_ops_page.py`에 인증된 미리보기 API/UI를 연결하고 `race_ops_preview.py`와 필요한 검사만 추가한다. 공통 원본은 수정하지 않는다. 변경 목록과 기준 SHA는 `changes.json`, `original_sha256.json`, `changes.patch`에 보존한다. 주행 제어기·guard·장치 드라이버·센서 교정은 변경하지 않는다.

## 사용

기존 `race_ops` HTTP 패널을 연 뒤 차량 이름과 기존 operator/view 토큰으로 연결한다. 새 서버나 별도 인증 경로는 없다. 아래 미리보기에서 라이다/카메라, 실제 발행 중인 토픽, 라이다 메시지 형식을 명시적으로 선택하고 **미리보기 적용**을 누른다. 기본은 모두 OFF다. **모두 끄기** 또는 마지막 설정 탭의 상태 조회가 3초 동안 끊기면 구독·캐시를 해제한다. 마지막 적용 탭 하나가 미리보기 설정과 조회 수명을 소유하며, 다른 조회 탭은 그 수명을 연장하지 않는다.

- 주행/시뮬 기본 토픽: native JSON `/scan/native`, LaserScan `/scan`, RGB `/camera/color/image_raw`.
- 기존 실물 preview 토픽은 `/hardware_preview/c1/native`, `/hardware_preview/camera/image_raw`이다. runtime 입력은 실행 시 실제 선택한 토픽을 넣는다. 패널이 장치를 개방하거나 토픽 발행기를 시작하지 않는다.
- 라이다는 유효 원측정 반환 최대720개를 최대5Hz로 표시한다. 기본 표시 반경3m는 조절할 수 있다. 점에 포인터를 놓으면 원측정 거리와 센서 x/y를 읽는다. 센서 frame·수신 나이·무효/무반환 수를 표시한다. 점 사이를 채우지 않으며 벽·차량 분류, 운동 보정, 차량 장착 좌표를 보장하지 않는다. NaN/무반환은 자유 공간의 증거가 아니다.
- 카메라는 `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8` Image를 지원한다. 최신 입력 한 개, 최대 가로640px·2Hz JPEG다. 수신 나이1초 초과는 stale이며 JPEG를 새로 제공하지 않는다. 원영상은 저장하지 않는다. 취득시각 검증을 주장하지 않는다.
- OFF 시 ROS 구독·인코딩 작업이 없고 첫 빈 canvas 이후 반복 그리기도 없다. ON 콜백은 최신 입력 참조만 교체하며 JSON 처리/JPEG 인코딩은 단일 worker에서 수행한다. 입력마다 작업을 쌓지 않는다.
- preview API는 operator/view 토큰 모두 허용한다. view 토큰의 경기·수동 명령 거절은 유지한다. 미리보기는 `/drive` publisher·enable·ARM·STOP 상태를 바꾸지 않는다.

## 검증과 이관

`test_overlay/`는 검사용 사본이며 설치 경로가 아니다. `check_page.js panel_preview.html`은 Node의 합성 DOM 검사로 실제 브라우저 시각 확인과 구분한다. `ros_preview_probe.py`는 실제 ROS 및 패널 자체 HTTP에 합성 native10Hz/RGB20Hz와 별도 LaserScan을 제공해 구독·JPEG·권한·OFF·조회 만료를 검사한다. Jetson·카메라/C1 포트·실물 출력은 사용하지 않는다. 결과와 한계는 `RESULT.md`를 따른다.

확인된 실행 환경은 **WSL 배포판 `Ubuntu-24.04`, ROS Jazzy, domain231**이다. `Ubuntu`라는 배포판은 없으며 domain234는 현재 FastDDS의 포트 범위를 넘는다. 현재 성공 명세는 `run_ros_v2.sh`, 새 경로 `/home/jinhyeong/it-arena-r434-preview-b`, 입력 `ros_sources_v2.zip`이다. 공유 WSL 소유권을 배정받은 뒤 Windows에서 아래 형식으로 실행한다. 기존 결과 경로에 재실행하지 않으며 후속 실행은 새 경로/스크립트로 원 결과를 보존한다.

```powershell
wsl -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/operator_preview_r434_v1/run_ros_v2.sh'
```
