# r461 운영 패널 브라우저 확인

## 범위와 결과

현재 공통 `race_ops_page.py`를 그대로 불러오는 Windows 합성 HTTP 서버를 만들어 실제 IAB 브라우저에서 확인했다. 상단에 합성 입력임을 표시했고 ROS·Jetson·MCU·실제 센서는 연결하지 않았다. 기존 r429/r434의 ROS 검사와 별도인 화면/입력 확인이다.

- 별도 개발 모드 없이 속도 상한이 보인다. `0.9 km/h` 편집이 `0.25 m/s`로 변환됐고, 적용 전 서버값 `1.5 m/s`와 편집값을 구분했다. 적용 버튼 뒤 합성 API에 `0.25`가 전달되고 서버 표시가 갱신됐다.
- 자율 구성에서 수동 단발 버튼은 비활성이다. 신호 대기 진입 뒤 수동 출발이 활성화됐으며 출발 후에는 다시 비활성화됐다. 이 상태 전이는 fixture가 제공한 것이며 실제 신호/MCU 판단의 추가 검증이 아니다.
- 별도 수동 구성에서 `0.36 km/h → 0.1 m/s`, 실행 `1.5 s`, 조향 `+5°`, 예상 거리 `0.150 m`를 확인했다. 준비와 실행을 각각 눌렀고 `/api/manual` 수신 값도 일치했다. 실제 구동은 없다.
- 합성 2m 벽의91개 점과640×360 시험 이미지를 표시했다. 이미지 자체는 PNG fixture이며 실물 카메라/JPEG 인코더 검증이 아니다. 미리보기 ON/OFF 전환을 확인했다.
- 서버 종료 뒤 연결 끊김 표시와 수동 준비/실행 비활성, STOP 버튼 유지를 확인했다. 실제 Wi-Fi/차체 정지 확인이 아니다.

## 발견·수정 준비

`hidden` 속성을 지정해도 `.preview img { display:block }`가 브라우저 기본 숨김 스타일을 덮어, OFF에서 빈 이미지 요소가 표시됐다. 실제 DOM 검사에서 원본 `hidden=""`이면서 `isVisible=true`였고, `.preview img[hidden]{display:none}`를 추가한 stage에서는 OFF=false/ON=true/OFF=false를 확인했다. 기능 구독이나 센서가 꺼지지 않는 문제로 해석하지 않는다.

또 화면의 ‘차량 재배치 시 먼저 RESET’ 문장을 현재 운영 규칙에 맞춰 ‘정지를 유지한 채 상태만 초기화하려면 RESET’으로 바꿨다. 기존 START/RESET 로직은 변경하지 않았다.

두 변경은 [stage_manifest.json](stage_manifest.json)의 사본에서 브라우저 확인을 마친 뒤, r457 주행·입력 회수가 끝나고 기존 공통 SHA를 대조해 **공통 통합**했다([영수증](integration.json)). 화면 변경에 새 ROS/주행 반복은 하지 않았다.

## 근거

- [합성 서버](serve_fixture.py), [실제 버튼 API 수신](requests.jsonl), [수정 전 서버](server_before.json), [수정본 서버](server.json)
- [수동 실행 후 연결 종료](manual_disconnected_before.jpg), [미리보기 ON](preview_on_after.jpg), [미리보기 OFF](preview_off_after.jpg)
- 두 서버 모두 정상 종료했으며 장치/ROS 프로세스를 생성하지 않았다.
