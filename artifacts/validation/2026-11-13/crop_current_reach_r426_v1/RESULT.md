# r426 저장 current 비교 결과

2026-11-13. **29개 단위검사와 고유 current 313개 저장 비교를 완료했다.** 원 baseline 양수 127개가 candidate 237개로 늘었고, 110개가 새 단일 재검사에서 양수로 회복했다. 이는 저장 계산 결과이며 새 부모 발행·guard/MCU 적용·실제 진행 성공이 아니다. root는 검토 후 공통 5파일에 exact 이관했고 [integration.json](integration.json)에 기록했다.

## 실제 확인

- `verify_v1/conditional-full.zip`의 `results/units/junit.xml`: tests29, failures0, errors0, skipped0. native build 및 실행 subprocess는 exit0/회수 완료다.
- 대표 seq1003/index312는 baseline0→candidate0.2 m/s였다. 원 경로·조향·취득시각을 유지하고 전체 motion query1,572개 중 unknown366개를 다시 검사했다. whole source113..118/양 flank112·119, 현재 방향 폭5.04°, R+P=0.299590m, 원점 diameter 약0.000172mm, 횡폭 상계26.345mm로 수락했다. 이 횡폭은 표본 방향 구간에 수직인 제한값이며 숨은 물체 크기나 연속 안전의 상계가 아니다.
- 대표를 재사용하고 나머지312개만 비교했다. baseline313개는 raw evaluator/checked target exact 및 부모 cap 후 기록 대조를 모두 통과했다. 비대상167개는 전체 출력 exact, 새 후보 거절35개는 새 진단을 제외한 원 필드 exact, 미진입1개는 전체 exact였다.
- 대표/나머지 × baseline/candidate의 네 Lane 모두 `exitcode=0`, `forced=false`, `alive=false`다. 각 행의 부모/child 입력 불변, 원 baseline 호출1회, 외부 project import0을 확인했다. 각 stage의 `source_changes=[]`, `preparation_changes=[]`, `verification_passed=true`다. 결과 생성 이후의 허가된 RESULT/integration 추가는 원 preparation manifest를 수정하지 않는다.

## 남은 35+1

35개는 모두 core의 현재 기하 검사에서는 양수였으나 최종 proof가 `different_current_whole_component`로 거절했다. 실제 query가 계획 crop 성분 이외의 별도 1칸 unknown 성분에도 걸렸다. 이 범위를 새로 허용하지 않고 0을 보존했다.

| 원 crop 전체 성분 | 현재에서 함께 걸린 추가 성분 | 건수 |
|---|---|---:|
| 114..118 | 120 | 7 |
| 111..115 | 117 | 4 |
| 113..117 | 119 | 9 |
| 112..116 | 118 | 15 |

미진입1개는 seq925/index273의 실제 측정속도 `−0.061225042014404256 m/s`가 후보의 0..0.6m/s 범위 밖이었기 때문이다. 이를 0으로 고치거나 이전 값을 대입하지 않았다. 원 dt는 약0.098s이고, same-scan 분류는 true였다.

## 범위와 보존

새 scope는 부모가 소유한 전체 frozen scan 동일성, verified native simulation, crop ON/r396 ON에 한정한다. 현재 history·기하·raw 전체 성분은 새로 검사했고 과거 허가를 재사용하지 않았다. 실물/host-estimated 입력으로 확대하지 않았으며 기본 OFF를 유지한다. 원 r418 정체·r423 0 결과는 그대로다. plan 전체 상태 이력, 실제 시간 만료, 독립 guard/MCU 적용, 전체 랩이나 실차는 이번 저장 비교로 검증하지 않았다.

기준 해시:

| 자료 | SHA256 |
|---|---|
| bundle_sources.zip | `e1476867bea21952c25aed173daddc5326cc333c4d37a50c86d544cf2d79d8f5` |
| preparation_manifest.json | `f61edcc58f7672a91626392a6b05f6833f5038311d8b701be423b6d15d3b5cda` |
| verify_v1/conditional-full.zip | `3c03b2b05bca757fa2fe6c5efe9ebe2df7d28a50d829235591e1c90740eabaaf` |
| verify_v1/conditional-full.json | `e0d1f20e8bc5c5bffcf456949e709ba4a6d57812836d9c17985dafc5184796fa` |
| verify_v1/representative.json | `e07fd0bbf81d9ca0f72b7616feb30417f565d4ef5a681ee7232f311579a6104a` |
| verify_v1/units.json | `f06c3231cb4292d8d9bcb2400f14f975e88d1cf2062642a12c67628cfbbc2a24` |
| integration.json | `3601b209a06ca4ca58830a2c63e554a7fda5061d94bdeea2971a678388c9f07e` |
