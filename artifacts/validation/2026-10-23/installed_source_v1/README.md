# 소스·설치본 일치 및 별도 빌드

- 기존 `install/local_setup.bash`만 읽고 `/tmp`에서 검사했다. 기존 Python은 egg-link/build 경로로 이어지며 **161개 파일/entry-point 대조 통과**였다. 오래된 코드 사용이나 설치 누락을 발견한 것은 아니다.
- 기존 설치 경로를 덮어쓰지 않고 `build/install_audit_20261023*`에 일반 복사형 빌드/설치를 수행했다. `arena_autonomy`, `arena_vehicle_interface`, `arena_description`, `arena_gazebo`, `arena_bringup` **5개 패키지 성공, 57.6 s**.
- 새 overlay만 읽은 `/tmp` 실행에서 Python 모듈과 share 경로가 새 설치 폴더 내부임을 확인했다. **166개 파일/경로/entry-point 검사 통과**. ROS 초기화/노드 생성은 검사 중 금지했고 entry point는 import만 했다.
- 새 빌드의 CTest `single_motor_equations` **1/1 통과**. 전체 소스 해시 전후 일치, `exit.txt=0`.
- Python 패키지 stderr는 `PYTHONDONTWRITEBYTECODE=1` 때문에 `.pyc` 생성을 생략한다는 경고뿐이었다. 오류를 무시해 성공 처리한 것은 아니다.
- Gazebo 주행/센서 노드 실행·모터 출력·Jetson 배포·실차 검증은 하지 않았다. 기존 운영 설치본/실행 기본값은 유지했다.

`existing.json`, `fresh.json`, `build.log`, `ctest.log`, `sources_*.sha256`, `source_diff.txt`가 근거다. 4,656회귀와 별도 설치/빌드 검사이며 개수를 합산하지 않는다.
