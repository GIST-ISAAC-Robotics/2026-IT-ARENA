# 센서 기록 전 읽기 전용 환경 점검

관련 74회귀·0.58 s, 설치된 PC Python 3.12/Jazzy의 5종 메시지·메타데이터·MCAP 등록 API·경로/디스크 검사 exit 0·소스 불변. 지정 capture 디렉터리가 생성되지 않은 것을 shell에서 확인했다. ROS 초기화/노드·장치·writer를 만들지 않으며 선택 보고서만 생성했다.

입력 API 미설치·metadata/writer/MCAP 미지원·잘못된 직렬화·작은 디스크·기존/외부 경로·Python 3.9·구동 토픽을 거절하는 반례를 검사했다. Humble API 모양은 stub이고 실제 설치본은 Jazzy다. plugin 등록이 실제 파일 열기/쓰기 성공이나 센서 연결·QoS 일치·시각 정확성·구동 준비를 뜻하지 않는다.

4,538 전체 회귀 이후의 추가 코드이므로 그 수에 합산하거나 포함됐다고 소급하지 않는다. `sources_before/after.txt`·tests.log·junit·report.json을 보존했다.
