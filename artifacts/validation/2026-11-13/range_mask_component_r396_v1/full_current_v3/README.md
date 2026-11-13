# r396 현재223개 제한 저장 비교 v3 — 실행 준비

원 `mixed_components_v2`를 독립 검토한 뒤 동일 후보를 저장223개에만 확대한다. [검토](REVIEW.md)에서 P1/P2 결함은 발견하지 못했다. 이 폴더의 하네스와 자료만 새로 작성했다. 공통 소스, r398/r400, 원 r386/v1/v2 및 원 결과는 수정하지 않았다. Windows에서 AST·해시·기대값 자료 무결성만 확인했으며 WSL/native/후보/ROS/주행 실행은 하지 않았다.

## 고정 비교

- 원 r386 `revision_final_geometry_v1/bundle_sources.zip` SHA256 `bf53444bbed329082c904d046921656e0035f9c1715c18e6e11812fb48cbfd52`의534개 소스/입력만 새 root에 추출한다. 현재 common이나 새 r399 운영 프로필을 import하지 않는다.
- r377 `follow_decisions.jsonl`의 current223개를 원 순서 그대로 읽는다. 양수119개를 포함한 전체 baseline과 결합 후보를 각1회 계산한다. 별도 helper-only arm, original795/legacy795, 기존29단위검사는 실행하지 않는다.
- 저장 cfg에서 윤거 `.1239`, 바퀴 반경 `.0325`, 바퀴폭 `.0261`만 최종 YAML 값으로 명시한다. length/width/wheelbase/송신 조향한계는 원 값과 exact 일치해야 한다. r377 marker `.14`에 대응한 동결5구간 마스크를 유지한다. r399 marker `.09`/4구간과 섞지 않는다.
- baseline223개는 원 final `rows.jsonl`에 보존된 `before`와 전체 출력 exact 비교한다. 123/125/222의 결합 후보 출력도 v2 결과 전체와 exact 비교한다. 경로 좌표·reason·진단 필드 삭제나 tolerance 비교는 없다. 비용 필드는 과거 출력의 일부가 아니며 새 실제 경과시간으로 별도 기록한다.
- 후보는 원 r386 거리 제안 함수와 최종 검사 그대로이고, 한 호출 동안 gap helper만 v2 원문으로 선택한다. 기존 baseline을 다시 호출하지 않는다. 수락 시 단1회 목표/최종 검사, 원 시각·경로·조향 보존, unknown 미충전 등을 확인한다. 비적용 출력은 진단 키만 제외하고 baseline과 exact, 비대상은 진단 키까지 exact여야 한다.
- 마지막222 입력을 원 생성기로 변형한3개만 보완한다: `current_time_unverified`는 장면 생성 거절/후보 미평가, `expired_path_source`와 `multiple_front_and_side_obstacles`는 baseline/후보를 계산하여 후보 비적용·0속도를 확인한다. 전자는 실제 STOP 명령 검증이 아니다.

원 r386 pipeline의 `passed=false`는 그대로다. 새 report는 원 실패를 성공으로 소급하지 않고 `original_r386_pipeline_passed=false`를 명시한다. 원시 입력과 상세 JSONL은 기존 Git 제외 정책을 유지한다. 이 폴더의 `expected_final_223.jsonl`은 원 산출물에서 추출한 재현 기대값이며 신규 알고리즘 결과가 아니다.

## 지정 실행자 명령

WSL 소유권을 받은 담당자만 아래1회 실행한다. 새 root/receipt가 이미 있으면 거절하며 자동 삭제·재시도하지 않는다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/range_mask_component_r396_v1/full_current_v3/execute_isolated.py \
  --r386-archive artifacts/validation/2026-11-13/current_corner_budget_r386_v1/revision_final_geometry_v1/bundle_sources.zip \
  --root /home/jinhyeong/it-arena-r396-full-20261113-c \
  --receipt artifacts/validation/2026-11-13/range_mask_component_r396_v1/full_current_v3/verify_v1
```

native build120초 + 비교300초 상한이다. timeout/비정상 종료/expected 불일치/반례 실패/소스 변경이면 즉시 실패로 기록하고 그 실행을 재시도하지 않는다. 각 자식 프로세스 그룹의 PID·종료코드를 보존하고 timeout은 TERM5초 뒤 필요시 KILL5초로 회수한다. 장치·ROS·Gazebo를 열지 않는다.

`verify_v1/execution.json` 및 `results.zip`을 회수한다. zip 안 `results/report.json`에서223 baseline exact, 세 이전 출력 exact, 비대상/비적용 불변 수, 양수 전후/회복 index,3반례의 서로 다른 평가 단계, source 변경0을 확인한다. `results/rows.jsonl`은 행별 원/새 전체 출력과 실제 경과시간이다. 추가 비용은 all/eligible/applied/non-target별 p50/p95/max 및100/200ms 초과 수를 보고한다. 이는 x86 단회 직렬 저장 비교이며 Jetson 비용·명령 만료 통과·실제 제어 주기 충족 판정이 아니다.

준비 파일과 원 결과 참조 해시는 `manifest.json`에 있다. 기존534개와 새 입력/스크립트의 실행 전후 해시를 확인한다. 실제1회 주행 여부와 채택은 이 결과를 검토한 뒤 별도로 결정한다.
