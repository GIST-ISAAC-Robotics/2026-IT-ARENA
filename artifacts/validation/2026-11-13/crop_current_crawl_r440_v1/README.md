# r440 단일0.1m/s crop 후속 재검증

root가 공통 `race_runtime_policy/crop_current_candidate.py`에 구현한 단일 저속 후속을 별도 동결 사본에서 검증했다. 검증 담당은 공통 코드를 수정하지 않았다. 대상 SHA256은 `886f4875a11cc5f7cc0af4924a99717a323464542ffb4fa48e45f218fe01d9e5`다. 변경 전 파일은 `original_crop_current_candidate.py`에 보존됐다.

**기존29개+추가6개=35검사와 현재 전체 Evaluator409개 저장 비교를 통과했다.** r43796개 중72개만0.1m/s로 바뀌고24개 거절은 exact 보존됐다. r426313개는 양수237개를 포함해 전체 exact 보존됐다. 새 주행·ROS·장치 출력은 실행하지 않았다.

## 실행과 비교 범위

초기에는 WSL 고장 지시에 따라 Windows 의존성과 BuildTools 존재만 확인했다. 검사/컴파일을 시작하기 전에 root가 WSL을 복구해 반환했으므로 Windows 새 native 빌드는 하지 않았다. 이후 검증은 전부 **WSL `Ubuntu-24.04`, 기존 Linux native binary, 전용 `/home/jinhyeong/it-arena-r440-verify-a`**에서 한 번 수행했다. Python fallback이나 가짜 native 선택으로 판정을 대체하지 않았다. r438 baseline 기록을 재사용하고 r440 전체 Evaluator만 계산했으며 같은409개를 두 환경에서 중복 실행하지 않았다.

`validation_sources_v1.zip`/`manifest.json`은 현재 패키지·native binary·시험·입력237파일의 원 SHA를 보존한다. 현재Evaluator 결과의 외부 project import0, 저장입력 변경0, 동결파일 변경0을 확인했다. 변경72개의 새0.1 proof/region 결과는 r438 고정0.1 실험과 exact 일치한다. 실제 core 호출을 관찰해 목표 `[0.2,0.1]` 두 번만 수행되고 실제 측정속도가 그대로 전달되는 것을 확인했다. 비대상에는 추가0.1 호출이 없었다. 공통 proof·기하·1cm 한계를 완화하거나 후보 검색 루프를 추가하지 않았다.

## 합성 음성 사례

새 검사6개는 깨끗한 seq643 한 개와 아래5개 반례다. 모두 현재 전체Evaluator와 실제 native 계산 함수를 호출했다. 원 positive/crop 계획 표식을 의도적으로 남긴 채 입력을 바꿔, 그 표식이 새로운 현재 측정 검사를 대신하지 못하는지 확인했다. 이는 실제 장치 관측이나 새 계획/주행 성공이 아니다.

| 입력 변화 | 실제 결과 |
|---|---|
| 원 seq643 그대로 | 0.1m/s, .2/.1 두 전체 재검사, 현재 crop proof 통과 |
| 전방 ±10° 안 유한 반환0.12m | `rebase_current_gap_hold`, clearance0, 최종0 |
| crop 성분 밖 측방80–125° 유한 반환0.12m | `rebase_current_gap_hold`, clearance0, 최종0 |
| 기존 crop의 인접 flank를 +Inf로 바꿔 미관측 확대 | `wide_nonreturn`, 최종0, 저속 수락 없음 |
| submitted reference를 current보다0.01초 뒤로 불일치 | `rebase_path_source_expired_or_invalid`, 최종0 |
| current epoch만 불일치 | `rebase_epoch_or_side_changed`, 최종0 |

기존29개 검사는 r426의 원 검사 파일과 당시 명시한 r423 scope6개 선택을 그대로 재사용했다. 원시 점유/근접 flank/NaN/거리 하한/열린·넓은·중앙 gap, 전체 성분 대응, 원점/방향 기하, 기존 양수 보존, 시각·부모 소유 scan 계약 등을 포함한다. 단위검사0.57초와 저장검사17.47초는 이번 WSL 실행 기록이다.

## 계산 비용과 남은 범위

r43796개의 현재 전체Evaluator 비용은 중앙값68.98ms/p95 78.78ms/max105.40ms다. **seq795 한 개가100ms를 넘었다.** 이를 숨기거나 초기화 비용으로 단정하지 않는다. r426313개는 중앙값24.85ms/p95 55.18ms/max92.00ms다. Windows 수치와 혼합하지 않았고, ROS/IPC·실제 명령 만료·Jetson·실차 부하를 포함하지 않는다. 다음 승인된 실제 실행에서 시간 여유와0.1 명령의 발행/보호/진행을 확인해야 한다. 이번 결과 자체로100ms deadline이나 주행 개선을 보장하지 않는다.

이번 범위에서 기능 결함은 발견하지 않았다. 현 구현은 원 .2 proof 성공+후측방 자체 mask만 거절+전방 mask0+실측0..0.15m/s+요청 목표0.1초과에만 후속을 붙이고, .1에서도 core 점유/제동과 전체 영역/proof를 다시 검사한다. 실패하면 원0 결과를 보존한다. 실제 r437 개선72개는 측정속도0.0m/s 사례이며, 더 넓은 실차 속도/환경 검증으로 확장하지 않는다.

## 원시 근거와 종료

- [verify_v1/execution.json](verify_v1/execution.json): 단위/저장 subprocess 모두 exit0, forced=false, reaped=true.
- [verify_v1/unit_result/junit.xml](verify_v1/unit_result/junit.xml):35검사·실패0.
- [verify_v1/unit_result/synthetic_evidence.json](verify_v1/unit_result/synthetic_evidence.json): 합성6개 전체 결과.
- [verify_v1/saved_result/report.json](verify_v1/saved_result/report.json), [rows.jsonl](verify_v1/saved_result/rows.jsonl):409개 결과·원시 비용.
- [handoff_audit.json](handoff_audit.json): 동결237파일·공통 target·원 r437 파일 보존·소유 프로세스 감사.

r438에서 WSL 오류로 종료후PID감사를 못 했던 원 기록은 그대로 유지한다. r440의 새 정상 종료/감사 결과를 과거 실행에 소급하지 않는다. 소유권 반환 이후 새 실행은 root의 자원 배정에 따른다.
