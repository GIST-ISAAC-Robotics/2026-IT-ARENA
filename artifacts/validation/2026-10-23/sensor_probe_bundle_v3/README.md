# 사전점검·간격 진단 포함 휴대 묶음 v3

관련 **249회귀·2.20 s**, 분리 소스 **147회귀·0.76 s** 통과. 19파일·38,553 B·18 Python 파일의 Python 3.10 문법 및 목록/해시를 검사했다. archive는 `build/sensor_probe_tools_20261023_v3.tar.gz`, 전달용 목록 버전은 `sensor_probe_tools_version=2`다. 산출물 실행 번호 v3와 목록 형식 버전은 다르다.

새 `build/sensor_probe_v3_isolated_42u_j8z4`에서 `python3 -I -B`로 핵심 모듈이 묶음 내부인지 확인했다. 기존 15파일 v2 archive도 원본 변경 없이 새 검증기로 통과했다. 실제 Python 3.10·ARM 실행은 아니다.

추가 `runtime_check.sh`는 설치된 `/opt/ros/jazzy`와 분리 묶음만 사용했다. 사전점검·기존 5입력 MCAP 감사·간격 진단은 exit 0, 미완료 MCAP 감사는 exit 1이며 잘못된 보고서를 생성하지 않았다. 노드 초기화/생성을 금지한 채 읽기만 수행했고 소스/입력 파일 해시는 불변이다. Jetson 전송·실물 실행·센서 시각/교정·실제 신규 기록 성능 검증은 하지 않았다.

상세 실행·검사·원본 해시는 이 디렉터리의 manifest/report/log에 있다. archive·분리 복사본·원시 입력은 Git 제외, 운영 기본값은 변경하지 않았다.
