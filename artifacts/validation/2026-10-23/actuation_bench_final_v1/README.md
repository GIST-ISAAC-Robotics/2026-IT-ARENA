# 출력 비활성 연결부까지 포함한 PC 전체 회귀

- [실행 스크립트](run_checks.sh)는 수신 API/생산 API/수신기/명령/링크/프레임의 C++ 대조 드라이버 여섯 개를 ASan/UBSan으로 새로 빌드하고 환경변수를 명시했다.
- [전체 회귀](full_tests.log): **3,923 passed·0 skipped·exit 0**, 141.84초. [컴파일 로그](compiler.log)는 비어 있으며 [소스 해시 차이](source_hash_diff.txt) 0 B다.
- 이 결과에는 출력 비활성 endpoint와 시간/보관/원천/제한 교환 후보가 들어 있다. **후속 `actuation_host_session.py` 추가 이전**의 결과다. 새 후보까지 검사한 수치로 소급하지 않는다.
- `source_snapshot.tar.gz`와 [대조 바이너리 해시](binaries.sha256)를 보존했다. 실제 보드 실행·물리 STOP·ROS/Gazebo 폐루프는 수행하지 않았다.
