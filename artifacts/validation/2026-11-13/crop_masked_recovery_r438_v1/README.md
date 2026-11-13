# r438 crop + 자체 후측방 마스크 저장 입력 비교

**제안: 기존 0.2m/s crop 검사 이후, crop proof를 통과하고 자체 후측방 마스크만 거절된 정지 근처 입력에 0.1m/s를 한 번만 재검사한다.** 이번 산출물은 비교·설계 근거이며 공통 제어기에는 연결하지 않았다. 새 주행·ROS·장치 실행은 없다.

## 비교 결과

원 r437 current96개와 r426 current313개를 r437 동결 코드/native binary로 계산했다. 원 baseline409개는 기존 기록과 exact 일치했다. 입력 SHA, 원측정 속도·scan·history·시각·조향·경로·차체 기하·1cm 한계를 유지하고 별도 stage의 고정 목표만 바꿨다.

| r437 96개 고정 목표 | 최종 양수 수락 | crop proof 통과 | 후측방 마스크 거절 | proof 실패 거절 | masked tail 범위 |
|---|---:|---:|---:|---:|---:|
| 0.2m/s 원 목표 | 0 | 72 | 72 | 24 | 11.798–13.670mm |
| 0.15m/s | 1 | 72 | 71 | 24 | 9.773–11.297mm |
| 0.1m/s | 72 | 72 | 0 | 24 | 7.065–8.160mm |

0.15m/s에서 통과한 것은 seq785 한 개다. 0.1m/s에서 회복한72개는 모두 원0.2 proof를 이미 통과했던 사례다. 별도 성분 등으로 proof가 실패한24개는 계속0이다. `unobserved`를 지우거나 자유 공간으로 채우지 않았다. 전체96개의 원 unknown 표본 수는 목표별368–432 / 294–344 / 182–212개로 남고, 새 이동영역에서 기존 동일 crop proof를 다시 수행했다. 전방 mask 표본은 모두0이다.

대표 seq643:

| 목표 | 이동영역 거리 | masked tail | unknown 질의 | proof | 최종 |
|---|---:|---:|---:|---|---|
| 0.2m/s | 0.073800m | 0.013422m | 429 | 통과 | 왼쪽 자체 mask 거절 |
| 0.15m/s | 0.053475m | 0.011122m | 343 | 통과 | 같은 거절 |
| 0.1m/s | 0.034400m | 0.008052m | 212 | 통과 | 0.1m/s |

r426313개는 후속 대상0개이며 전체 출력 exact 보존이다. 기존 양수237개(이전127개와 r426 crop 추가110개)를 모두 유지한다. 낮은 목표를 미리 고르는 단순 정지 근처/crop 조건은 기존 양수110개에도 걸린다. 이110개는 현재0.1m/s보다 큰 양수이므로 사전0.1 상한은 기존 성능 보존 조건에 맞지 않는다.

## 연결을 검토할 최소 범위

기존 `same_scan_scope`의 verified native simulation/crop ON/r396 ON 계약을 그대로 사용한다. 원 crop0.2 검사 결과가 `speed=0`, `reason=route_current_large_motion_region_unobserved`, crop diagnostic `current_crop_full_region_rejected`, `proof.accepted=true`, `blocked=large_motion_masked_toward_left|right`, `masked_front_samples=0`, 유효 query>0인 때만 후보로 분류한다. 실제 측정속도는 기존 SOLO 저속 계약과 같은0..0.15m/s 이하로 제한하되, **이번 r43796개에서 실제 검증한 값은 전부0.0m/s**다. 음수/상한 초과값을 고치거나 대체하지 않는다. 원 checked target이0.1보다 작거나 같으면 추가하지 않는다.

해당 경우에만 목표 `min(original_plan,0.1m/s)` 하나로 기존 core 점유·제동·현재/직전 조향·source/TTL 검사를 반복한다. 같은 조향/경로/시각인지 확인하고 전체 motion_region과 현재 raw crop proof를 새로 계산한다. 기존1cm tail/전방 mask 판정이 통과해야 양수로 바꾸고, 어떤 거절이든 원0 출력을 보존한다. 0.15→0.1 단계 검색이나 더 작은 목표 반복은 제안하지 않는다. 초기0.2 성공·proof 실패·일반 미관측·전방 가림·양수 결정에는 추가 호출이 없다.

실제 유한 반환 장애물의 `rebase_current_gap_hold` 사례는 이번409개 안에0개다. 따라서 그 음성 사례가 새로 통과했다고 주장하지 않는다. 원 core의 현재 `occupied_endpoints_xy_m`·양 조향 swept clearance·제동/정지 판정은 그대로 호출했으며 변경하지 않았다. 이 코드 보존과 실제 장애물 음성 사례의 실행 증거를 구분한다. crop의 숨은 물체 잔여 위험, 표본 영역과 연속 충돌 안전의 차이, independent guard 요구도 유지한다.

## 계산 비용과 한계

WSL `Ubuntu-24.04`에서 ROS/IPC 없는 단일 Python 과정으로409개를 한 번 계산했다. r43796개 원 전체 current 검사의 중앙값49.04ms/p95 57.48ms, 고정0.15 추가 검사31.74ms/p95 34.81ms, 고정0.1 추가 검사21.13ms/p95 23.78ms/max24.29ms였다. 대상72개의 원 전체+단일0.1 추가 합계는 중앙값71.10ms/p95 80.80ms/max89.60ms다. 순서·캐시·WSL 환경에 영향을 받는 관측 비용이며 Jetson 실주행/IPC/명령 만료를 포함한 deadline 검증이 아니다. 별도 반복 성능시험은 하지 않았다.

원래 .2 proof를 생략하는 사전목표 결정은 기존 양수110개를 보존하기 어렵다. 단일 .1 후속은 계산이 추가되지만 범위가 명확하고 기존 출력 보존을 이번 자료에서 확인했다. 다음 구현은 이 제한된 후속만 검토하는 것이 작다. 이후 실제 발행/guard/명령 만료/진행은 별도 승인된 대표에서 확인할 사항이며 이번 저장 계산을 새 주행 성공으로 합산하지 않는다.

## 산출물과 실행 명세

- [result_v1/report.json](result_v1/report.json): 수용·거절·비용·import/입력 불변 집계.
- [result_v1/rows.jsonl](result_v1/rows.jsonl):409개 행의 원 baseline 및 각 고정 목표 결과.
- [summary.json](summary.json): 대표 seq643, 최소 연결 비교, Windows 원본 SHA 감사.
- [preparation.json](preparation.json): r437 원 source/trace, r426 원 입력/결과, package/native binary와 입력 SHA.
- `stage/fixed_target_candidate.py`: 동결 crop 함수에서 목표 상한 인자와 그 표기만 분리한 시험용 함수다. 공통 이관 파일이 아니다.

`preparation.json`의 `stage_sha256`는 Windows 파일 쓰기 전 LF 문자열 해시다. 실제 stage는 Windows 기본 줄바꿈 CRLF로 기록되어 byte SHA가 다르지만 LF 정규화 내용은 exact 일치한다. 원 manifest·stage는 고치지 않고 실제 byte SHA와 이 구분을 `summary.json.stage_source_audit`에 추가했다. 동결 원 package/native binary와 입력 파일 SHA는 byte 그대로 일치한다.

확인된 환경은 WSL `Ubuntu-24.04`, 전용 `/home/jinhyeong/it-arena-r438-crop-a`다. ROS domain·장치·기존 install을 사용하거나 바꾸지 않는다. `run_saved_v1.sh`는 새 경로에서만 실행하며 이미 실행한 경로에 재실행하지 않는다. 원 실행은 exit0, 동결 package 변경0·입력변경0·외부 project import0으로 끝났다. 결과 회수 후 추가 `/proc` 감사 호출은 `Wsl/Service/E_UNEXPECTED`로 두 번 거절됐다. 단일 foreground 계산의 exit0과 결과 회수는 확인했지만 그 이후 새 PID 목록은 확인하지 못했으며 WSL을 재시작하지 않았다.
