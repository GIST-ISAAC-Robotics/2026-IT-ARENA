# r396 고정 실행 인계

실행 담당: root가 배정한 Terra. 현재 WSL 작업의 종료/자원 반환 후 아래 한 묶음만 실행한다. 이 준비 단계에서 실행하지 않았다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
/usr/bin/python3 -B artifacts/validation/2026-11-13/range_mask_component_r396_v1/execute_isolated.py \
  --r386-archive 'artifacts/validation/2026-11-13/current_corner_budget_r386_v1/revision_final_geometry_v1/bundle_sources.zip' \
  --root '/home/jinhyeong/it-arena-r396-component-20261113-a' \
  --receipt 'artifacts/validation/2026-11-13/range_mask_component_r396_v1/verify_v1'
```

root/receipt는 존재하면 중단하며 자동 삭제·덮어쓰기하지 않는다. 실행 스크립트와 [manifest.json](manifest.json) SHA-256은 인계 메시지에 함께 전달한다. 스크립트는 원 bundle의 고정 SHA와 r396 manifest 파일 해시를 검증하고, 별도 root의534개 원 소스 및 r396 파일을 전후 확인한다.

실제로 실행할 하위 명령은 다음 세 개다. 모두 새 root의 `geometry`만 import하며 공통197파일 동결과 분리한다.

```text
geometry/src/arena_autonomy: /usr/bin/python3 -B setup.py build_ext --inplace
r396: /usr/bin/python3 -B -m pytest -q /home/jinhyeong/it-arena-r396-component-20261113-a/r396/test_component.py
r396: /usr/bin/python3 -B /home/jinhyeong/it-arena-r396-component-20261113-a/r396/compare_three.py --output /home/jinhyeong/it-arena-r396-component-20261113-a/r396/results
```

한 단계라도 실패하면 후속 실행을 하지 않는다. build/tests/compare 제한은120/120/180초이며 timeout 시 자기 child group에 TERM 후5초, 필요하면 KILL 후5초로 회수한다. 장치/WSL 공통 설치/Gazebo/ROS를 사용하지 않는다.

회수할 결과는 `verify_v1/execution.json`, `verify_v1/results.zip`이다. zip 안 tests.log, compare_three.log, results/report.json, row123/125/222 및 실제 질의 파일을 root가 독립 검토한다. `execution_completed`, source_changes, completed=3, baseline_exact, helper-only/결합 회복 index를 구분한다. 개선0이면 중단한다. timing은 단일 진단 비용으로만 읽고 처리율·실시간 성능이나 실차 효과로 승격하지 않는다.
