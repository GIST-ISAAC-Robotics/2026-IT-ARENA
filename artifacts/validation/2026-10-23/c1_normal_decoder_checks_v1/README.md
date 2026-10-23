# normal decoder 및 점별 보정 관련 회귀

`run_checks.sh`: 새 native handler 회귀와 기존 이동 장면/점별 투영/운동 보정 검사, **126 passed in 3.76s**, pytest/래퍼 exit 0. skip/실패 0. `tests.log`, `junit.xml`, `checks_exit.txt` 보존. 전후 `src/scripts/tests/config` 해시는 동일(`source_hash_diff.txt` 빈 파일), 당시 source snapshot은 기존 Git 제외 정책에 따라 로컬에 보존했다.

원문 SHA·실제 wire 자료형, 19개 분할 경계·고정 seed 200점, partial/reset, 지연식 분기·batching·잘못된 바이트/시각·입력 상한·기존 보정 관문을 확인했다. 실제 장치/ROS/주행 시험은 아니다. 원문 컴파일 경고는 가상 인터페이스의 미사용 인수 경고이며 `native/compile.stderr`와 pytest 임시 빌드에 남는다. 앞선 전체 회귀 수와 합산하지 않는다.
