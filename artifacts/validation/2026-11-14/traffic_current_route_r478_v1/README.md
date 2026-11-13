# r478 교통 대표 준비

**준비만 수행, 미실행.** 새 610 mm 코스에 등록된 `straight_moving_lead` 10초·seed 0 한 번을 r475 최신 차량/센서/시설/정책으로 준비한다. 앞차 0.30→0.58 m/s, 초기 중심 간격 1.6 m·오른쪽 0.12 m, LiDAR에 보이는 높이220 mm 상대다. 마커 observer OFF.

[선택 근거·범위·고정 명령](PLAN.md), [고정 CLI](arguments.json), [각본](config.json), [원본 보존/복사 관계](provenance.json). 원 r6 정체 실패는 그대로 보존한다. 완주·낮은 차량·실차 검증이 아니며 성공까지 반복하지 않는다.

[Windows 정적 대조](static_check.json)는 원 case 동일, 현재 공통 입력1,323개 변경0, actor XML 보존·시설3개 정확한 합성·contact plugin1개, 알려진 CLI25개와 Python12파일 구문을 확인했다. 실제 공통 평가기 import·경로 기하 검사·주행은 아직 수행하지 않았다. [recipe 해시](recipe_manifest.json)를 최종 freeze에 사용한다.

WSL은 r477 담당이 소유한다. root 검토·자원 인계 후 최종 freeze와 1회 실행을 진행한다. 공통 `src/scripts/config/tests` 수정·빌드·Jetson 접근은 없다.
