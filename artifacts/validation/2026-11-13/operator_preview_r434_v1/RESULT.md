# r434 검증 상태

- Windows 순수44검사 통과: preview11 + r429 수동19 + 기존 운영14. 유효 반환/무반환 구분, 최대720점, RGB 크기·색, OFF 무작업, 최신1개, OFF 후 늦게 완료한 인코딩 폐기, 조회 소유자·만료·stale를 확인했다.
- Node 합성 DOM 검사 통과: r429 단위/편집값/수동 제한 회귀, canvas 범위·무반환 표시·OFF 반복 그리기 없음, stale, 기존 토큰을 사용하는 설정/JPEG 및 끄기 동작. 실제 브라우저 시각 검증은 미수행이다.
- 실제 ROS·기존 인증 HTTP 대표는 `Ubuntu-24.04`/domain231/root-b에서 통과했다. 1.050초 구간 native10개·RGB21개를 수신하고 JPEG2개를 처리했다. 전체 카메라 수신23개/encode4개, 최대 pending 전체2개(종류별 최신1개), camera1개다. native725개 중 유효722·무효3·표시720, LaserScan498점, JPEG640×362를 확인했다.
- 명시 OFF 뒤 native/scan/RGB subscriber 모두0, 조회3초 만료 해제, stale JPEG404, 무인증 JPEG401, view 토큰의 경기 명령403을 확인했다. 기존 `/drive`·`/drive/safe` publisher를 추가하지 않았다. 실행기 오류0, worker/spin 종료, 소유PID0으로 WSL을 반환했다.
- 실제 결과는 [root-b result](ros_it-arena-r434-preview-b/result_v2/result.json), [소스/프로세스 감사](ros_source_audit.json)에 있다. a/b의 동결224파일은 각각 변경0이다. 첫 실행의 `Ubuntu` 배포판 미존재는 ROS 시작 전 실패이며, 올바른 배포판에서 domain234가 DDS 포트 상한으로 거절된 원 로그는 [root-a](ros_it-arena-r434-preview-a/run_v1.log)에 보존했다. 소스 변경 없이 domain231의 새 root-b로 재실행했다.
- 공통 소스·설치·장치·주행 시험은 변경/실행하지 않았다. 이미지 원본·JPEG 파일 저장 경로를 추가하지 않았고 결과에 영상 데이터를 넣지 않는다. 이 짧은 WSL 합성 입력 대표를 Jetson 실주행 부하0·물리 센서 정확도·실차 성능 검증으로 확대하지 않는다. 실제 브라우저 시각 검증은 여전히 미수행이다.
